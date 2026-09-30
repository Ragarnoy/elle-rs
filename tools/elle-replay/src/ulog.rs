//! Minimal ULog reader: formats, subscriptions and data records, which is all
//! the firmware writes (`crates/elle-ulog`). Every other record type is skipped.

use std::collections::HashMap;

use anyhow::{Context, Result, bail};

const MAGIC: [u8; 7] = [0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    U64,
    I64,
    F32,
    F64,
}

impl Kind {
    fn parse(t: &str) -> Result<Self> {
        Ok(match t {
            "uint8_t" | "bool" | "char" => Self::U8,
            "int8_t" => Self::I8,
            "uint16_t" => Self::U16,
            "int16_t" => Self::I16,
            "uint32_t" => Self::U32,
            "int32_t" => Self::I32,
            "uint64_t" => Self::U64,
            "int64_t" => Self::I64,
            "float" => Self::F32,
            "double" => Self::F64,
            _ => bail!("unsupported ULog field type {t}"),
        })
    }

    const fn size(self) -> usize {
        match self {
            Self::U8 | Self::I8 => 1,
            Self::U16 | Self::I16 => 2,
            Self::U32 | Self::I32 | Self::F32 => 4,
            Self::U64 | Self::I64 | Self::F64 => 8,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Field {
    kind: Kind,
    offset: usize,
    count: usize,
}

/// One message type's layout.
#[derive(Clone, Debug, Default)]
pub struct Format {
    fields: HashMap<String, Field>,
    size: usize,
}

impl Format {
    fn parse(body: &str) -> Result<(String, Self)> {
        let (name, rest) = body.split_once(':').context("format without ':'")?;
        let mut f = Self::default();
        for decl in rest.split(';').filter(|d| !d.is_empty()) {
            let (ty, field) = decl.split_once(' ').context("field without a name")?;
            let (base, count) = match ty.split_once('[') {
                Some((b, n)) => (b, n.trim_end_matches(']').parse()?),
                None => (ty, 1),
            };
            let kind = Kind::parse(base)?;
            f.fields.insert(
                field.to_string(),
                Field {
                    kind,
                    offset: f.size,
                    count,
                },
            );
            f.size += kind.size() * count;
        }
        Ok((name.to_string(), f))
    }
}

/// Records of one message type (payloads without the message id).
#[derive(Clone, Debug)]
pub struct Series {
    format: Format,
    pub records: Vec<Vec<u8>>,
}

impl Series {
    fn field(&self, name: &str, kind: Kind) -> Field {
        let f = *self
            .format
            .fields
            .get(name)
            .unwrap_or_else(|| panic!("no field {name}"));
        assert_eq!(f.kind, kind, "field {name} has another type");
        f
    }

    fn bytes<'a>(&self, rec: &'a [u8], f: Field, i: usize) -> &'a [u8] {
        let at = f.offset + i * f.kind.size();
        &rec[at..at + f.kind.size()]
    }

    #[must_use]
    pub fn u64(&self, rec: &[u8], name: &str) -> u64 {
        let f = self.field(name, Kind::U64);
        u64::from_le_bytes(self.bytes(rec, f, 0).try_into().unwrap())
    }

    #[must_use]
    pub fn u32(&self, rec: &[u8], name: &str) -> u32 {
        let f = self.field(name, Kind::U32);
        u32::from_le_bytes(self.bytes(rec, f, 0).try_into().unwrap())
    }

    #[must_use]
    pub fn i16(&self, rec: &[u8], name: &str) -> i16 {
        let f = self.field(name, Kind::I16);
        i16::from_le_bytes(self.bytes(rec, f, 0).try_into().unwrap())
    }

    #[must_use]
    pub fn u8(&self, rec: &[u8], name: &str) -> u8 {
        self.bytes(rec, self.field(name, Kind::U8), 0)[0]
    }

    #[must_use]
    pub fn f32(&self, rec: &[u8], name: &str) -> f32 {
        self.f32s::<1>(rec, name)[0]
    }

    #[must_use]
    pub fn f32s<const N: usize>(&self, rec: &[u8], name: &str) -> [f32; N] {
        let f = self.field(name, Kind::F32);
        assert_eq!(f.count, N, "field {name} has {} elements", f.count);
        std::array::from_fn(|i| f32::from_le_bytes(self.bytes(rec, f, i).try_into().unwrap()))
    }

    /// A `uint8_t[N]` field.
    #[must_use]
    pub fn blob<'a>(&self, rec: &'a [u8], name: &str) -> &'a [u8] {
        let f = self.field(name, Kind::U8);
        &rec[f.offset..f.offset + f.count]
    }

    #[must_use]
    pub fn has(&self, name: &str) -> bool {
        self.format.fields.contains_key(name)
    }
}

/// A parsed log: message name → its records, plus the dropouts the writer noted.
#[derive(Debug, Default)]
pub struct ULog {
    pub series: HashMap<String, Series>,
    /// Dropout records: milliseconds lost in each.
    pub dropouts: Vec<u16>,
    /// The file ended inside a record (common after a power cut).
    pub truncated: bool,
}

impl ULog {
    pub fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < 16 || data[..7] != MAGIC {
            bail!("not a ULog file");
        }
        let mut formats: HashMap<String, Format> = HashMap::new();
        let mut ids: HashMap<u16, String> = HashMap::new();
        let mut log = Self::default();
        let mut at = 16;
        while at + 3 <= data.len() {
            let size = usize::from(u16::from_le_bytes([data[at], data[at + 1]]));
            let kind = data[at + 2];
            let Some(body) = data.get(at + 3..at + 3 + size) else {
                log.truncated = true;
                break;
            };
            at += 3 + size;
            match kind {
                b'F' => {
                    let (name, f) = Format::parse(std::str::from_utf8(body)?)?;
                    formats.insert(name, f);
                }
                b'A' if body.len() >= 3 => {
                    let id = u16::from_le_bytes([body[1], body[2]]);
                    let name = std::str::from_utf8(&body[3..])?.to_string();
                    let format = formats
                        .get(&name)
                        .with_context(|| format!("subscription to unknown {name}"))?
                        .clone();
                    log.series.entry(name.clone()).or_insert(Series {
                        format,
                        records: Vec::new(),
                    });
                    ids.insert(id, name);
                }
                b'D' if body.len() >= 2 => {
                    let id = u16::from_le_bytes([body[0], body[1]]);
                    if let Some(s) = ids.get(&id).and_then(|n| log.series.get_mut(n))
                        && body.len() - 2 >= s.format.size
                    {
                        s.records.push(body[2..].to_vec());
                    }
                }
                b'O' if body.len() >= 2 => {
                    log.dropouts.push(u16::from_le_bytes([body[0], body[1]]));
                }
                _ => {}
            }
        }
        Ok(log)
    }

    #[must_use]
    pub fn get(&self, name: &str) -> Option<&Series> {
        self.series.get(name)
    }
}
