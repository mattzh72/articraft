from __future__ import annotations

from agent.harness import ArticraftAgent


def test_post_success_design_audit_injected_once() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._post_success_design_audit_sent = False
    agent._post_success_design_audit_enabled = True
    agent.trace_writer = None

    conversation: list[dict] = []
    injected = agent._maybe_inject_post_success_design_audit(conversation)

    assert injected is True
    assert conversation
    content = conversation[0]["content"]
    assert "Compile passed. Do one brief final visual audit before concluding." in content
    assert "<design_audit>" in content
    assert "Focus on visual realism first" in content
    assert "Do not add more tests or `probe_model` calls" in content
    assert "conclude immediately" in content

    injected_again = agent._maybe_inject_post_success_design_audit(conversation)
    assert injected_again is False
    assert len(conversation) == 1


def test_post_success_design_audit_not_injected_when_disabled() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._post_success_design_audit_sent = False
    agent._post_success_design_audit_enabled = False
    agent.trace_writer = None

    conversation: list[dict] = []
    injected = agent._maybe_inject_post_success_design_audit(conversation)

    assert injected is False
    assert not conversation
