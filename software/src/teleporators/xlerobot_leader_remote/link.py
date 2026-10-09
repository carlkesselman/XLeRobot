"""Socket tuning shared by both ends of the leader link.

One function, applied to the PUSH on the operator station and the PULL on the
cart, so the two sides cannot be tuned differently by accident.

What it sets and why - each of these was arrived at by measurement on this
robot, not taken from a checklist:

TOS 0xC0 (CS6 / WMM AC_VO)
    The link stalled for ~300 ms about once a second whenever the cart was
    also streaming video to the operator's viewer, and never otherwise. The
    cart's Python was exonerated: its control loop held 29.5 Hz through the
    stalls, which a 300 ms GIL or GC freeze cannot do. The bytes were late
    on the wire. Mechanism: the cart's uplink carries the video, the cart's
    TCP ACKs back to the operator queue behind seconds of it, and the
    operator's tiny action flow - 60 packets/s, application-limited - has
    its congestion window pinned at the initial 10 segments, so after ~10
    unacknowledged messages (~170 ms) its kernel stops sending until an ACK
    gets through or the 200 ms minimum RTO fires. IP_TOS applies to the
    socket's pure ACKs as well as its data, and 0xC0 maps to the voice
    queue on Linux and most access points, so the actions and their ACKs
    jump the video's best-effort queue at every hop that honours WMM.

HEARTBEAT_* / TCP_KEEPALIVE_*
    With CONFLATE the pipe high-water mark is -1, so send() succeeds as long
    as a TCP connection exists at all - it says nothing about delivery. A
    peer that vanishes (cart rebooted, wifi dropped) is noticed only when
    the kernel exhausts tcp_retries2, about fifteen minutes, and the PULL
    side never reconnects because ZMTP sends nothing while idle. ZMTP
    heartbeats detect a dead peer in ~3 s and let the auto-reconnect fire;
    TCP keepalive is the belt to that brace.

Neither of these is a workaround for a bug; they are the difference between
a command channel and a best-effort one.
"""

import zmq

# DSCP CS6 in the high six bits. 0xB8 (EF) is the other common choice; CS6
# is what Linux' default skb_priority map puts in AC_VO without a tc rule.
_TOS_VOICE = 0xC0


def tune_socket(sock: "zmq.Socket") -> None:
    """Apply the link options. Call BEFORE bind()/connect()."""
    sock.setsockopt(zmq.TOS, _TOS_VOICE)

    sock.setsockopt(zmq.HEARTBEAT_IVL, 1000)      # ping every second
    sock.setsockopt(zmq.HEARTBEAT_TIMEOUT, 3000)  # no pong in 3 s: peer is gone
    sock.setsockopt(zmq.HEARTBEAT_TTL, 5000)      # tell the peer to expect us within 5 s

    sock.setsockopt(zmq.TCP_KEEPALIVE, 1)
    sock.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 1)
    sock.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
    sock.setsockopt(zmq.TCP_KEEPALIVE_CNT, 3)


# Keys the host adds to every message for the receiver's diagnostics. They
# are not action features and are stripped before the action is returned.
SEQ_KEY = "_seq"
TIME_KEY = "_t"
