from __future__ import annotations

import importlib

import pytest

import sdk_hybrid

cq = pytest.importorskip("cadquery")


@pytest.mark.parametrize(
    ("gear_ctor", "kwargs", "expected_type_name"),
    [
        ("SpurGear", {"module": 1.0, "teeth_number": 19, "width": 5.0, "bore_d": 5.0}, "Compound"),
        (
            "HerringboneGear",
            {"module": 1.0, "teeth_number": 24, "width": 8.0, "helix_angle": 20.0},
            "Solid",
        ),
        (
            "RingGear",
            {"module": 1.0, "teeth_number": 40, "width": 5.0, "rim_width": 2.0},
            "Solid",
        ),
        (
            "HerringboneRingGear",
            {
                "module": 1.0,
                "teeth_number": 48,
                "width": 8.0,
                "rim_width": 2.5,
                "helix_angle": 20.0,
            },
            "Solid",
        ),
        (
            "PlanetaryGearset",
            {
                "module": 1.0,
                "sun_teeth_number": 12,
                "planet_teeth_number": 9,
                "width": 4.0,
                "rim_width": 2.0,
                "n_planets": 3,
            },
            "Compound",
        ),
        (
            "HerringbonePlanetaryGearset",
            {
                "module": 1.0,
                "sun_teeth_number": 12,
                "planet_teeth_number": 9,
                "width": 6.0,
                "rim_width": 2.0,
                "n_planets": 3,
                "helix_angle": 20.0,
            },
            "Compound",
        ),
        (
            "BevelGear",
            {"module": 1.0, "teeth_number": 18, "cone_angle": 45.0, "face_width": 4.0},
            "Solid",
        ),
        (
            "BevelGearPair",
            {
                "module": 1.0,
                "gear_teeth": 24,
                "pinion_teeth": 16,
                "face_width": 4.0,
                "axis_angle": 90.0,
            },
            "Compound",
        ),
        ("RackGear", {"module": 1.0, "length": 20.0, "width": 5.0, "height": 3.0}, "Solid"),
        (
            "HerringboneRackGear",
            {"module": 1.0, "length": 20.0, "width": 5.0, "height": 3.0, "helix_angle": 20.0},
            "Solid",
        ),
        (
            "Worm",
            {"module": 1.0, "lead_angle": 20.0, "n_threads": 2, "length": 15.0},
            "Solid",
        ),
        (
            "CrossedHelicalGear",
            {"module": 1.0, "teeth_number": 17, "width": 5.0, "helix_angle": 20.0},
            "Solid",
        ),
        (
            "CrossedGearPair",
            {
                "module": 1.0,
                "gear1_teeth_number": 20,
                "gear2_teeth_number": 20,
                "gear1_width": 4.0,
                "gear2_width": 4.0,
                "shaft_angle": 90.0,
                "gear1_helix_angle": 30.0,
            },
            "Compound",
        ),
        (
            "HyperbolicGear",
            {"module": 1.0, "teeth_number": 17, "width": 5.0, "twist_angle": 20.0},
            "Solid",
        ),
        (
            "HyperbolicGearPair",
            {"module": 1.0, "gear1_teeth_number": 20, "width": 4.0, "shaft_angle": 60.0},
            "Compound",
        ),
    ],
)
def test_all_vendored_gears_build(
    gear_ctor: str,
    kwargs: dict[str, float | int],
    expected_type_name: str,
) -> None:
    gear_cls = getattr(sdk_hybrid, gear_ctor)
    gear = gear_cls(**kwargs)
    body = gear.build()

    assert isinstance(body, cq.Shape)
    assert type(body).__name__ == expected_type_name


def test_workplane_gear_plugin_and_top_level_helpers() -> None:
    spur = sdk_hybrid.SpurGear(module=1.0, teeth_number=19, width=5.0, bore_d=5.0)

    wp = cq.Workplane("XY").gear(spur)
    assert isinstance(wp, cq.Workplane)
    assert len(wp.vals()) == 1

    helper_wp = sdk_hybrid.gear(cq.Workplane("XY"), spur)
    assert isinstance(helper_wp, cq.Workplane)
    assert len(helper_wp.vals()) == 1

    union_wp = cq.Workplane("XY").circle(8.0).extrude(2.0).addGear(spur)
    assert isinstance(union_wp, cq.Workplane)
    assert union_wp.solids().vals()


def test_sdk_hybrid_gears_module_reexports_classes() -> None:
    from sdk_hybrid.gears import RingGear, SpurGear

    current_sdk_hybrid = importlib.import_module("sdk_hybrid")

    assert SpurGear.__name__ == current_sdk_hybrid.SpurGear.__name__
    assert RingGear.__name__ == current_sdk_hybrid.RingGear.__name__
