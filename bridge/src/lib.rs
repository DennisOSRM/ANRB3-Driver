//! The map bridge: what it serves the page, and what it keeps.
//!
//! [`http`] is the transport: HTTP and WebSocket, implemented here; gzip via
//! flate2 and TLS via rustls. [`map`] holds the aircraft and their tracks for
//! the quarter of an hour the page draws. [`hexdb`] looks up registration,
//! type, operator, route and photograph from hexdb.io and caches them on disk.

pub mod hexdb;
pub mod http;
pub mod map;

use std::sync::{Mutex, MutexGuard};

/// Lock a mutex, recovering it if a thread panicked while holding it. Nothing
/// in this crate leaves shared state half-built, so the contents are still
/// usable, and refusing to serve because an unrelated thread died would be
/// worse.
pub(crate) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    /// A mutex left poisoned by a thread that panicked while holding it still
    /// hands over its contents.
    #[test]
    fn a_poisoned_lock_is_recovered() {
        let m = Arc::new(Mutex::new(7));
        let m2 = Arc::clone(&m);
        let died = std::thread::spawn(move || {
            let _g = m2.lock().expect("the lock");
            panic!("a thread that dies holding the lock");
        })
        .join();
        assert!(died.is_err());
        assert!(m.is_poisoned());
        *lock(&m) += 1;
        assert_eq!(*lock(&m), 8);
    }
}

/// A certificate authority and a server certificate it signed, for the tests
/// that speak TLS to a listener on 127.0.0.1. The server certificate names
/// 127.0.0.1 and localhost, and both are valid until 2126.
#[cfg(test)]
pub(crate) mod test_tls {
    use std::sync::Arc;

    /// The authority, to be trusted by a test client.
    pub const CA: &str = "\
-----BEGIN CERTIFICATE-----
MIIBnDCCAUOgAwIBAgIUTtH4EqEpuHfGvmWad+crr8/hArAwCgYIKoZIzj0EAwIw
GzEZMBcGA1UEAwwQYW5yYi1tYXAgdGVzdCBDQTAgFw0yNjA5MjcwNjMyNTZaGA8y
MTI2MDkwMzA2MzI1NlowGzEZMBcGA1UEAwwQYW5yYi1tYXAgdGVzdCBDQTBZMBMG
ByqGSM49AgEGCCqGSM49AwEHA0IABHVpHVTaU4nGEIaz4HipOXXqLHjGRjmNMNTg
B7lKjuXLYX1p/6t7LmZnko0ijanu7256KKgeEznVgXDchYEGLaajYzBhMB0GA1Ud
DgQWBBSmjls4P+nzV69CGE6N7VTiHcKnmTAfBgNVHSMEGDAWgBSmjls4P+nzV69C
GE6N7VTiHcKnmTAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwICBDAKBggq
hkjOPQQDAgNHADBEAiAB75gPE/+RL5gQD++22/8B9Nf9OPtsjBCzOZYJm3ywvAIg
Kl68BvZSCfHcsraUfKykc4UTAZlOA2Cceol/fQPtNCI=
-----END CERTIFICATE-----
";

    /// The server certificate.
    pub const CERT: &str = "\
-----BEGIN CERTIFICATE-----
MIIBwjCCAWmgAwIBAgIUSocWAlSbrGmgUphyS5YKodB1B3IwCgYIKoZIzj0EAwIw
GzEZMBcGA1UEAwwQYW5yYi1tYXAgdGVzdCBDQTAgFw0yNjA5MjcwNjMyNTZaGA8y
MTI2MDkwMzA2MzI1NlowFDESMBAGA1UEAwwJMTI3LjAuMC4xMFkwEwYHKoZIzj0C
AQYIKoZIzj0DAQcDQgAEluSR4nMF1lCxKI8gWRoMeJcIqRHAQKv8oVcxcH2YtY5F
KdnlrXDXRG2To6ZwWptNhUIJC3JONYv0fK1RwWI68KOBjzCBjDAJBgNVHRMEAjAA
MA4GA1UdDwEB/wQEAwIHgDATBgNVHSUEDDAKBggrBgEFBQcDATAaBgNVHREEEzAR
hwR/AAABgglsb2NhbGhvc3QwHQYDVR0OBBYEFB9nUkH9+08wwhlXrbtMvIrt6XHH
MB8GA1UdIwQYMBaAFKaOWzg/6fNXr0IYTo3tVOIdwqeZMAoGCCqGSM49BAMCA0cA
MEQCIHFL0pt3yE/FG8fuSn+/Gl4pfmWbhoJnnmP9JWyOLSkNAiAmqVJQgGbDA1wi
JhbG4+7DQzvfW4fT69vLrTHhl6sI0A==
-----END CERTIFICATE-----
";

    /// The server certificate's private key, in PKCS#8.
    pub const KEY: &str = "\
-----BEGIN PRIVATE KEY-----
MIGHAgEAMBMGByqGSM49AgEGCCqGSM49AwEHBG0wawIBAQQgZlROUgowmTNiVeDr
noe0b5NCTtfwJrLWeBR4vy6odFOhRANCAASW5JHicwXWULEojyBZGgx4lwipEcBA
q/yhVzFwfZi1jkUp2eWtcNdEbZOjpnBam02FQgkLck41i/R8rVHBYjrw
-----END PRIVATE KEY-----
";

    /// A client configuration that trusts [`CA`] and nothing else.
    pub fn client() -> rustls::ClientConfig {
        let mut roots = rustls::RootCertStore::empty();
        for c in rustls_pemfile::certs(&mut CA.as_bytes()) {
            roots
                .add(c.expect("the test authority"))
                .expect("a usable authority");
        }
        rustls::ClientConfig::builder_with_provider(Arc::new(
            rustls::crypto::ring::default_provider(),
        ))
        .with_safe_default_protocol_versions()
        .expect("the default protocol versions")
        .with_root_certificates(roots)
        .with_no_client_auth()
    }

    /// A server configuration that presents [`CERT`].
    pub fn server() -> rustls::ServerConfig {
        let chain = rustls_pemfile::certs(&mut CERT.as_bytes())
            .collect::<Result<Vec<_>, _>>()
            .expect("the test certificate");
        let key = rustls_pemfile::private_key(&mut KEY.as_bytes())
            .expect("the test key")
            .expect("a key in the PEM text");
        rustls::ServerConfig::builder_with_provider(Arc::new(
            rustls::crypto::ring::default_provider(),
        ))
        .with_safe_default_protocol_versions()
        .expect("the default protocol versions")
        .with_no_client_auth()
        .with_single_cert(chain, key)
        .expect("a usable certificate and key")
    }
}
