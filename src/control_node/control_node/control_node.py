"""Compatibility entry point for the dedicated MPC runtime."""

from control_node.mpc_control_node import MpcControlNode, main

# Retain the historical class import for downstream scripts during migration.
ControlNode = MpcControlNode

__all__ = ["ControlNode", "MpcControlNode", "main"]
