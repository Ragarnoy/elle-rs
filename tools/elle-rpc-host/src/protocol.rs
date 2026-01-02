//! Postcard-RPC protocol encoding/decoding

use anyhow::{Context, Result};
use postcard_rpc::header::{VarHeader, VarKey, VarSeq};
use postcard_rpc::Endpoint;

/// Encode an RPC request with COBS framing
pub fn encode_request<E: Endpoint>(body: &E::Request) -> Result<Vec<u8>>
where
    E::Request: serde::Serialize,
{
    let mut buf = vec![0u8; 256];
    let header = VarHeader {
        key: VarKey::Key8(E::REQ_KEY),
        seq_no: VarSeq::Seq1(0),
    };

    let (header_slice, remaining) = header.write_to_slice(&mut buf).context("Header too large")?;
    let body_bytes = postcard::to_slice(body, remaining)?;
    let total_len = header_slice.len() + body_bytes.len();

    let mut encoded = vec![0u8; total_len + (total_len / 254) + 2];
    let encoded_len = cobs::encode(&buf[..total_len], &mut encoded);
    encoded.truncate(encoded_len);
    encoded.push(0x00);
    Ok(encoded)
}

/// Decode an RPC response from COBS-framed data
pub fn decode_response<E: Endpoint>(data: &[u8]) -> Result<E::Response>
where
    E::Response: serde::de::DeserializeOwned,
{
    let mut decoded = vec![0u8; data.len()];
    let report = cobs::decode(data, &mut decoded)?;
    let (header, body) =
        VarHeader::take_from_slice(&decoded[..report.frame_size()]).context("Invalid header")?;

    if header.key != VarKey::Key8(E::RESP_KEY) {
        anyhow::bail!("Unexpected response key");
    }
    postcard::from_bytes(body).context("Failed to deserialize")
}

/// Encode an RPC response (for mock transport)
pub fn encode_response<E: Endpoint>(body: &E::Response) -> Result<Vec<u8>>
where
    E::Response: serde::Serialize,
{
    let mut buf = vec![0u8; 256];
    let header = VarHeader {
        key: VarKey::Key8(E::RESP_KEY),
        seq_no: VarSeq::Seq1(0),
    };
    let (header_slice, remaining) = header.write_to_slice(&mut buf).context("Header too large")?;
    let body_bytes = postcard::to_slice(body, remaining)?;
    let total_len = header_slice.len() + body_bytes.len();

    let mut encoded = vec![0u8; total_len + (total_len / 254) + 2];
    let encoded_len = cobs::encode(&buf[..total_len], &mut encoded);
    encoded.truncate(encoded_len);
    Ok(encoded)
}
