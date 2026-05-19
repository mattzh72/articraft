from __future__ import annotations

import pytest

from agent.cost import calculate_cost, pricing_for_provider_model


def test_anthropic_latest_opus_pricing_uses_explicit_model_rates() -> None:
    pricing = pricing_for_provider_model("anthropic", "claude-opus-4-7")

    assert pricing == {
        "input_uncached": 5.00,
        "input_cached": 0.50,
        "input_cache_write": 6.25,
        "input_cache_write_1h": 10.00,
        "output": 25.00,
    }


def test_anthropic_latest_opus_pricing_calculates_cache_hits() -> None:
    pricing = pricing_for_provider_model("anthropic", "claude-opus-4-7")
    assert pricing is not None

    cost = calculate_cost(
        {
            "prompt_tokens": 1_000_000,
            "cached_tokens": 400_000,
            "candidates_tokens": 100_000,
            "total_tokens": 1_100_000,
        },
        pricing,
    )

    assert cost.input_uncached_cost == 3.0
    assert cost.input_cached_cost == 0.2
    assert cost.output_cost == 2.5
    assert cost.total_cost == pytest.approx(5.7)


def test_anthropic_latest_opus_pricing_calculates_cache_writes() -> None:
    pricing = pricing_for_provider_model("anthropic", "claude-opus-4-7")
    assert pricing is not None

    cost = calculate_cost(
        {
            "prompt_tokens": 1_000_000,
            "cached_tokens": 400_000,
            "cache_creation_input_tokens": 100_000,
            "candidates_tokens": 100_000,
            "total_tokens": 1_100_000,
        },
        pricing,
    )

    assert cost.input_uncached_cost == pytest.approx(3.125)
    assert cost.input_cached_cost == 0.2
    assert cost.output_cost == 2.5
    assert cost.total_cost == pytest.approx(5.825)


def test_anthropic_latest_opus_pricing_calculates_one_hour_cache_writes() -> None:
    pricing = pricing_for_provider_model("anthropic", "claude-opus-4-7")
    assert pricing is not None

    cost = calculate_cost(
        {
            "prompt_tokens": 1_000_000,
            "cache_creation_input_tokens": 100_000,
            "cache_creation_5m_input_tokens": 40_000,
            "cache_creation_1h_input_tokens": 60_000,
            "candidates_tokens": 100_000,
            "total_tokens": 1_100_000,
        },
        pricing,
    )

    assert cost.input_uncached_cost == pytest.approx(4.5 + 0.25 + 0.6)
    assert cost.output_cost == 2.5
    assert cost.total_cost == pytest.approx(7.85)


def test_anthropic_sonnet_and_haiku_pricing_are_available() -> None:
    assert pricing_for_provider_model("anthropic", "claude-sonnet-4-6") == {
        "input_uncached": 3.00,
        "input_cached": 0.30,
        "input_cache_write": 3.75,
        "input_cache_write_1h": 6.00,
        "output": 15.00,
    }
    assert pricing_for_provider_model("anthropic", "claude-haiku-4-5") == {
        "input_uncached": 1.00,
        "input_cached": 0.10,
        "input_cache_write": 1.25,
        "input_cache_write_1h": 2.00,
        "output": 5.00,
    }


def test_gpt55_pricing_uses_explicit_model_rates() -> None:
    pricing = pricing_for_provider_model("openai", "gpt-5.5-2026-04-23")

    assert pricing == {
        "input_uncached": 5.00,
        "input_cached": 0.50,
        "output": 30.00,
        "prompt_tier_threshold_tokens": 272_000,
        "input_uncached_above_threshold": 10.00,
        "input_cached_above_threshold": 1.00,
        "output_above_threshold": 45.00,
    }


def test_gpt55_pricing_uses_high_context_tier_above_threshold() -> None:
    pricing = pricing_for_provider_model("openai", "gpt-5.5-2026-04-23")
    assert pricing is not None

    cost = calculate_cost(
        {
            "prompt_tokens": 300_000,
            "cached_tokens": 100_000,
            "candidates_tokens": 10_000,
            "total_tokens": 310_000,
        },
        pricing,
    )

    assert cost.input_uncached_cost == 2.0
    assert cost.input_cached_cost == 0.1
    assert cost.output_cost == 0.45
    assert cost.total_cost == pytest.approx(2.55)


def test_gpt54_pricing_remains_unchanged() -> None:
    pricing = pricing_for_provider_model("openai", "gpt-5.4")

    assert pricing is not None
    assert pricing["input_uncached"] == 2.50
    assert pricing["input_cached"] == 0.25
    assert pricing["output"] == 15.00
