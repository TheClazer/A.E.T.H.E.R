"""
A.E.T.H.E.R trust HUD  —  Streamlit dashboard (the cockpit "trust light").

Run on the Ubuntu box AFTER sourcing the workspace:
    streamlit run src/aether_health_cockpit/aether_health_cockpit/streamlit_app.py

Spins a background rclpy node, subscribes the integrity topics, and renders a
live green/amber/red trust light, the nav state, the breathing horizontal PL,
and a rolling trust trace. (Salvaged-and-repurposed from the original AETHER
dashboard instinct — now wired to a real navigation-integrity bus.)
"""
import threading
import collections

try:
    import streamlit as st
except ImportError:  # allow import without streamlit installed (CI lint)
    st = None

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from aether_msgs.msg import ProtectionLevel


class HudListener(Node):
    def __init__(self):
        super().__init__('aether_hud')
        self.trust = 1.0
        self.state = 'NOMINAL'
        self.hpl = 0.0
        self.create_subscription(Float32, '/nav/trust', lambda m: setattr(self, 'trust', m.data), 10)
        self.create_subscription(String, '/nav/state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(ProtectionLevel, '/nav/integrity_bound',
                                 lambda m: setattr(self, 'hpl', m.horizontal_pl), 10)


def _spin(node):
    rclpy.spin(node)


def color(trust):
    return '#2F7D4F' if trust >= 0.8 else ('#C98A12' if trust >= 0.4 else '#C42A1C')


def main():
    if st is None:
        print('streamlit not installed; run: pip install streamlit'); return
    if not rclpy.ok():
        rclpy.init()
    if 'node' not in st.session_state:
        st.session_state.node = HudListener()
        threading.Thread(target=_spin, args=(st.session_state.node,), daemon=True).start()
        st.session_state.hist = collections.deque(maxlen=200)

    node = st.session_state.node
    st.set_page_config(page_title='A.E.T.H.E.R Trust HUD', layout='wide')
    st.title('A.E.T.H.E.R — navigation integrity HUD')
    st.session_state.hist.append(node.trust)

    c1, c2, c3 = st.columns(3)
    c1.markdown(f"<h1 style='color:{color(node.trust)}'>{node.trust:.2f}</h1>"
                "<p>TRUST 0–1</p>", unsafe_allow_html=True)
    c2.markdown(f"<h1 style='color:{color(node.trust)}'>{node.state}</h1>"
                "<p>NAV STATE</p>", unsafe_allow_html=True)
    c3.markdown(f"<h1>{node.hpl:.2f} m</h1><p>HORIZONTAL PROTECTION LEVEL</p>",
                unsafe_allow_html=True)
    st.line_chart(list(st.session_state.hist))
    st.caption('When the camera dies, the bound breathes and the truth stays inside — '
               'and we know within half a second.')
    st.autorefresh = st.experimental_rerun if hasattr(st, 'experimental_rerun') else None


if __name__ == '__main__':
    main()
