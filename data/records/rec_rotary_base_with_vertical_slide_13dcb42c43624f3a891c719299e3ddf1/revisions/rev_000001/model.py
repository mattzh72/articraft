from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    """CadQuery box in meters, centered at a local point."""
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _union_all(*items: cq.Workplane) -> cq.Workplane:
    shape = items[0]
    for item in items[1:]:
        shape = shape.union(item)
    return shape


def _annulus(outer_radius: float, inner_radius: float, height: float, z_min: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_welding_positioner")

    model.material("machine_black", rgba=(0.015, 0.017, 0.020, 1.0))
    model.material("turret_blue", rgba=(0.05, 0.10, 0.16, 1.0))
    model.material("bearing_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    model.material("guide_bronze", rgba=(0.72, 0.52, 0.25, 1.0))
    model.material("dark_covers", rgba=(0.02, 0.025, 0.030, 1.0))
    model.material("ram_silver", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("safety_red", rgba=(0.80, 0.08, 0.05, 1.0))
    model.material("weld_yellow", rgba=(0.90, 0.62, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.68, 0.68, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material="machine_black",
        name="base_slab",
    )
    base.visual(
        mesh_from_cadquery(_annulus(0.255, 0.185, 0.045, 0.095), "stator_ring"),
        material="bearing_steel",
        name="stator_ring",
    )

    turret = model.part("turret_frame")
    turret.visual(
        mesh_from_cadquery(_annulus(0.245, 0.190, 0.026, 0.0), "rotating_ring"),
        material="bearing_steel",
        name="rotating_ring",
    )

    casting = _union_all(
        _box((0.48, 0.48, 0.070), (0.0, 0.0, 0.061)),
        _box((0.050, 0.260, 0.630), (-0.145, -0.050, 0.410)),
        _box((0.050, 0.260, 0.630), (0.145, -0.050, 0.410)),
        _box((0.345, 0.260, 0.060), (0.0, -0.050, 0.695)),
        _box((0.345, 0.260, 0.060), (0.0, -0.050, 0.120)),
        _box((0.260, 0.050, 0.585), (0.0, 0.082, 0.400)),
        _box((0.225, 0.037, 0.550), (0.0, -0.176, 0.430)),
        _box((0.060, 0.170, 0.025), (0.195, -0.030, 0.585)),
        _box((0.035, 0.060, 0.055), (0.235, -0.090, 0.585)),
    )
    turret.visual(
        mesh_from_cadquery(casting, "turret_casting"),
        material="turret_blue",
        name="turret_casting",
    )

    guide_rails = _union_all(
        _box((0.018, 0.022, 0.560), (-0.055, -0.188, 0.430)),
        _box((0.018, 0.022, 0.560), (0.055, -0.188, 0.430)),
    )
    turret.visual(
        mesh_from_cadquery(guide_rails, "guide_rails"),
        material="bearing_steel",
        name="guide_rails",
    )

    left_pads = _union_all(
        _box((0.034, 0.050, 0.115), (-0.057, -0.216, 0.300)),
        _box((0.034, 0.050, 0.115), (-0.057, -0.216, 0.610)),
    )
    right_pads = _union_all(
        _box((0.034, 0.050, 0.115), (0.057, -0.216, 0.300)),
        _box((0.034, 0.050, 0.115), (0.057, -0.216, 0.610)),
    )
    turret.visual(
        mesh_from_cadquery(left_pads, "left_guide_pads"),
        material="guide_bronze",
        name="left_guide_pads",
    )
    turret.visual(
        mesh_from_cadquery(right_pads, "right_guide_pads"),
        material="guide_bronze",
        name="right_guide_pads",
    )

    travel_stops = _union_all(
        _box((0.035, 0.036, 0.028), (0.074, -0.216, 0.198)),
        _box((0.035, 0.036, 0.028), (0.074, -0.216, 0.696)),
    )
    turret.visual(
        mesh_from_cadquery(travel_stops, "travel_stops"),
        material="safety_red",
        name="travel_stops",
    )

    cable_bracket = _union_all(
        _box((0.040, 0.090, 0.018), (-0.187, -0.035, 0.560)),
        _box((0.030, 0.030, 0.060), (-0.220, -0.080, 0.560)),
        _box((0.050, 0.018, 0.018), (-0.205, -0.095, 0.590)),
    )
    turret.visual(
        mesh_from_cadquery(cable_bracket, "cable_bracket"),
        material="dark_covers",
        name="cable_bracket",
    )

    ram = model.part("ram")
    ram.visual(
        Box((0.055, 0.044, 0.700)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material="ram_silver",
        name="ram_body",
    )
    ram.visual(
        Box((0.049, 0.008, 0.460)),
        origin=Origin(xyz=(0.0, -0.026, 0.230)),
        material="dark_covers",
        name="front_cover",
    )
    ram.visual(
        Box((0.085, 0.065, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.516)),
        material="dark_covers",
        name="top_cap",
    )
    ram.visual(
        Box((0.090, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, -0.030, 0.390)),
        material="weld_yellow",
        name="weld_mount",
    )
    ram.visual(
        Box((0.0125, 0.040, 0.270)),
        origin=Origin(xyz=(-0.03375, 0.0, 0.155)),
        material="dark_covers",
        name="left_wear_shoe",
    )
    ram.visual(
        Box((0.0125, 0.040, 0.270)),
        origin=Origin(xyz=(0.03375, 0.0, 0.155)),
        material="dark_covers",
        name="right_wear_shoe",
    )

    model.articulation(
        "turret_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.8, lower=-3.14159, upper=3.14159),
    )
    model.articulation(
        "ram_lift",
        ArticulationType.PRISMATIC,
        parent=turret,
        child=ram,
        origin=Origin(xyz=(0.0, -0.220, 0.310)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1400.0, velocity=0.15, lower=0.0, upper=0.260),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    turret = object_model.get_part("turret_frame")
    ram = object_model.get_part("ram")
    turret_yaw = object_model.get_articulation("turret_yaw")
    ram_lift = object_model.get_articulation("ram_lift")

    ctx.check(
        "separate yaw and lift joints",
        turret_yaw.articulation_type == ArticulationType.REVOLUTE
        and ram_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"turret={turret_yaw.articulation_type}, ram={ram_lift.articulation_type}",
    )

    with ctx.pose({turret_yaw: 0.0, ram_lift: 0.0}):
        ctx.expect_gap(
            turret,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="rotating_ring",
            negative_elem="stator_ring",
            name="bearing rings seat without penetration",
        )
        ctx.expect_overlap(
            turret,
            base,
            axes="xy",
            min_overlap=0.20,
            elem_a="rotating_ring",
            elem_b="stator_ring",
            name="concentric bearing rings carry the yaw turret",
        )
        ctx.expect_gap(
            ram,
            turret,
            axis="x",
            min_gap=0.008,
            positive_elem="ram_body",
            negative_elem="left_guide_pads",
            name="ram clears left guide pads",
        )
        ctx.expect_gap(
            turret,
            ram,
            axis="x",
            min_gap=0.008,
            positive_elem="right_guide_pads",
            negative_elem="ram_body",
            name="ram clears right guide pads",
        )
        ctx.expect_contact(
            ram,
            turret,
            contact_tol=0.0005,
            elem_a="left_wear_shoe",
            elem_b="left_guide_pads",
            name="left wear shoe is captured by guide pad",
        )
        ctx.expect_contact(
            ram,
            turret,
            contact_tol=0.0005,
            elem_a="right_wear_shoe",
            elem_b="right_guide_pads",
            name="right wear shoe is captured by guide pad",
        )
        ctx.expect_overlap(
            ram,
            turret,
            axes="z",
            min_overlap=0.38,
            elem_a="ram_body",
            elem_b="guide_rails",
            name="collapsed ram remains captured in rails",
        )

    rest_pos = ctx.part_world_position(ram)
    with ctx.pose({turret_yaw: 0.65, ram_lift: 0.0}):
        ctx.expect_gap(
            turret,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="rotating_ring",
            negative_elem="stator_ring",
            name="yaw bearing clearance is unchanged when rotated",
        )

    with ctx.pose({turret_yaw: 0.0, ram_lift: 0.260}):
        ctx.expect_overlap(
            ram,
            turret,
            axes="z",
            min_overlap=0.30,
            elem_a="ram_body",
            elem_b="guide_rails",
            name="extended ram retains guide engagement",
        )
        ctx.expect_gap(
            ram,
            turret,
            axis="x",
            min_gap=0.008,
            positive_elem="ram_body",
            negative_elem="left_guide_pads",
            name="extended ram clears left guide pads",
        )
        ctx.expect_gap(
            turret,
            ram,
            axis="x",
            min_gap=0.008,
            positive_elem="right_guide_pads",
            negative_elem="ram_body",
            name="extended ram clears right guide pads",
        )
        extended_pos = ctx.part_world_position(ram)

    ctx.check(
        "positive lift raises the Z ram",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.24,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
