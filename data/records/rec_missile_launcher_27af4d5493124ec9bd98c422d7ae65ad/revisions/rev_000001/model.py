from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_loop(center_a: float, center_b: float, size_a: float, size_b: float) -> list[tuple[float, float]]:
    half_a = size_a * 0.5
    half_b = size_b * 0.5
    return [
        (center_a - half_a, center_b - half_b),
        (center_a + half_a, center_b - half_b),
        (center_a + half_a, center_b + half_b),
        (center_a - half_a, center_b + half_b),
    ]


def _build_canister_shell_mesh():
    outer_profile = _rect_loop(0.0, 0.0, 0.88, 1.08)
    hole_profiles = [
        _rect_loop(height_center, width_center, 0.30, 0.40)
        for height_center in (-0.19, 0.19)
        for width_center in (-0.24, 0.24)
    ]
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        1.60,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_y(pi / 2)
    return mesh_from_geometry(geom, "four_cell_canister_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trailer_launcher")

    olive = model.material("olive", rgba=(0.36, 0.40, 0.25, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.24, 0.28, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.42, 0.45, 0.48, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    chassis = model.part("chassis")
    deck = Box((3.40, 2.10, 0.16))
    chassis.visual(
        deck,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        material=olive,
        name="deck",
    )
    chassis.visual(
        Box((3.65, 0.18, 0.22)),
        origin=Origin(xyz=(0.02, -0.78, 0.47)),
        material=dark_olive,
        name="left_frame_rail",
    )
    chassis.visual(
        Box((3.65, 0.18, 0.22)),
        origin=Origin(xyz=(0.02, 0.78, 0.47)),
        material=dark_olive,
        name="right_frame_rail",
    )
    chassis.visual(
        Box((2.15, 0.14, 0.16)),
        origin=Origin(xyz=(-0.55, 0.0, 0.47)),
        material=dark_olive,
        name="front_crossmember",
    )
    chassis.visual(
        Box((2.15, 0.14, 0.16)),
        origin=Origin(xyz=(0.95, 0.0, 0.47)),
        material=dark_olive,
        name="rear_crossmember",
    )
    chassis.visual(
        Box((1.55, 0.12, 0.12)),
        origin=Origin(xyz=(-2.20, -0.42, 0.49), rpy=(0.0, 0.0, 0.24)),
        material=dark_olive,
        name="left_drawbar",
    )
    chassis.visual(
        Box((1.55, 0.12, 0.12)),
        origin=Origin(xyz=(-2.20, 0.42, 0.49), rpy=(0.0, 0.0, -0.24)),
        material=dark_olive,
        name="right_drawbar",
    )
    chassis.visual(
        Box((0.22, 0.12, 0.10)),
        origin=Origin(xyz=(-2.95, 0.0, 0.48)),
        material=steel,
        name="coupler_head",
    )
    chassis.visual(
        Box((1.70, 0.90, 0.10)),
        origin=Origin(xyz=(-2.05, 0.0, 0.50)),
        material=dark_olive,
        name="tongue_plate",
    )
    chassis.visual(
        Cylinder(radius=0.08, length=1.92),
        origin=Origin(xyz=(0.35, 0.0, 0.37), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    chassis.visual(
        Cylinder(radius=0.42, length=0.22),
        origin=Origin(xyz=(0.35, -1.07, 0.37), rpy=(pi / 2, 0.0, 0.0)),
        material=rubber,
        name="left_wheel",
    )
    chassis.visual(
        Cylinder(radius=0.42, length=0.22),
        origin=Origin(xyz=(0.35, 1.07, 0.37), rpy=(pi / 2, 0.0, 0.0)),
        material=rubber,
        name="right_wheel",
    )
    chassis.visual(
        Box((0.18, 0.18, 0.50)),
        origin=Origin(xyz=(-1.95, 0.0, 0.25)),
        material=steel,
        name="landing_jack",
    )
    chassis.inertial = Inertial.from_geometry(deck, mass=2800.0, origin=Origin(xyz=(0.0, 0.0, 0.64)))

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.22, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=steel,
        name="pedestal",
    )
    turntable.visual(
        Cylinder(radius=0.44, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=dark_olive,
        name="slew_ring",
    )
    turntable.visual(
        Box((1.25, 0.90, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=olive,
        name="turntable_platform",
    )
    turntable.visual(
        Box((0.30, 1.26, 0.14)),
        origin=Origin(xyz=(-0.28, 0.0, 0.53)),
        material=dark_olive,
        name="lower_yoke_brace",
    )
    turntable.visual(
        Box((0.20, 0.12, 1.00)),
        origin=Origin(xyz=(-0.17, -0.64, 0.86)),
        material=olive,
        name="left_yoke_arm",
    )
    turntable.visual(
        Box((0.20, 0.12, 1.00)),
        origin=Origin(xyz=(-0.17, 0.64, 0.86)),
        material=olive,
        name="right_yoke_arm",
    )
    turntable.inertial = Inertial.from_geometry(
        Box((1.25, 0.90, 1.12)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
    )

    model.articulation(
        "chassis_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=turntable,
        origin=Origin(xyz=(0.45, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.7),
    )

    canister_pack = model.part("canister_pack")
    canister_pack.visual(
        _build_canister_shell_mesh(),
        origin=Origin(xyz=(0.44, 0.0, 0.0)),
        material=olive,
        name="canister_shell",
    )
    canister_pack.visual(
        Box((0.08, 0.92, 0.76)),
        origin=Origin(xyz=(-0.32, 0.0, 0.0)),
        material=dark_olive,
        name="rear_bulkhead",
    )
    canister_pack.visual(
        Box((0.12, 1.04, 0.14)),
        origin=Origin(xyz=(-0.20, 0.0, -0.37)),
        material=dark_olive,
        name="base_spine",
    )
    canister_pack.visual(
        Cylinder(radius=0.07, length=0.04),
        origin=Origin(xyz=(0.0, -0.56, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    canister_pack.visual(
        Cylinder(radius=0.07, length=0.04),
        origin=Origin(xyz=(0.0, 0.56, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    canister_pack.inertial = Inertial.from_geometry(
        Box((1.68, 1.12, 0.88)),
        mass=1400.0,
        origin=Origin(xyz=(0.44, 0.0, 0.0)),
    )

    model.articulation(
        "turntable_to_canister_pack",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=canister_pack,
        origin=Origin(xyz=(-0.02, 0.0, 1.08)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.5, lower=0.0, upper=1.10),
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.42, 0.03, 0.28)),
        origin=Origin(xyz=(0.21, 0.015, 0.0)),
        material=steel,
        name="panel_door",
    )
    service_panel.visual(
        Box((0.18, 0.004, 0.08)),
        origin=Origin(xyz=(0.24, 0.032, 0.02)),
        material=dark_olive,
        name="panel_label_pad",
    )
    service_panel.visual(
        Box((0.04, 0.012, 0.02)),
        origin=Origin(xyz=(0.31, 0.036, 0.0)),
        material=dark_olive,
        name="panel_latch",
    )
    for barrel_name, barrel_z in (
        ("upper_hinge_barrel", 0.10),
        ("center_hinge_barrel", 0.0),
        ("lower_hinge_barrel", -0.10),
    ):
        service_panel.visual(
            Cylinder(radius=0.018, length=0.05),
            origin=Origin(xyz=(0.0, 0.015, barrel_z)),
            material=steel,
            name=barrel_name,
        )
    service_panel.inertial = Inertial.from_geometry(
        Box((0.42, 0.03, 0.28)),
        mass=45.0,
        origin=Origin(xyz=(0.21, 0.015, 0.0)),
    )

    model.articulation(
        "canister_pack_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=canister_pack,
        child=service_panel,
        origin=Origin(xyz=(0.22, 0.54, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.2, lower=0.0, upper=1.35),
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

    chassis = object_model.get_part("chassis")
    turntable = object_model.get_part("turntable")
    canister_pack = object_model.get_part("canister_pack")
    service_panel = object_model.get_part("service_panel")
    turret_joint = object_model.get_articulation("chassis_to_turntable")
    elevation_joint = object_model.get_articulation("turntable_to_canister_pack")
    panel_joint = object_model.get_articulation("canister_pack_to_service_panel")

    ctx.check(
        "turntable joint uses vertical axis",
        tuple(round(v, 6) for v in turret_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={turret_joint.axis}",
    )
    ctx.expect_contact(
        turntable,
        chassis,
        elem_a="pedestal",
        elem_b="deck",
        contact_tol=1e-6,
        name="turntable pedestal sits on trailer deck",
    )
    ctx.check(
        "elevation joint raises with positive motion",
        tuple(round(v, 6) for v in elevation_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={elevation_joint.axis}",
    )
    ctx.check(
        "service panel joint uses vertical side hinge axis",
        tuple(round(v, 6) for v in panel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={panel_joint.axis}",
    )
    ctx.expect_contact(
        canister_pack,
        turntable,
        elem_a="left_trunnion",
        elem_b="left_yoke_arm",
        contact_tol=1e-6,
        name="left trunnion is carried by the yoke",
    )
    ctx.expect_contact(
        canister_pack,
        turntable,
        elem_a="right_trunnion",
        elem_b="right_yoke_arm",
        contact_tol=1e-6,
        name="right trunnion is carried by the yoke",
    )
    ctx.expect_gap(
        service_panel,
        canister_pack,
        axis="y",
        max_gap=1e-6,
        max_penetration=0.0,
        positive_elem="panel_door",
        negative_elem="canister_shell",
        name="service panel closes flush against canister side",
    )

    pack_front_rest = ctx.part_element_world_aabb(canister_pack, elem="canister_shell")
    turret_rest = ctx.part_element_world_aabb(canister_pack, elem="canister_shell")
    panel_rest = ctx.part_world_aabb(service_panel)
    with ctx.pose({elevation_joint: 0.80}):
        pack_front_raised = ctx.part_element_world_aabb(canister_pack, elem="canister_shell")
    with ctx.pose({turret_joint: pi / 2}):
        turret_turned = ctx.part_element_world_aabb(canister_pack, elem="canister_shell")
    with ctx.pose({panel_joint: 1.10}):
        panel_open = ctx.part_world_aabb(service_panel)

    ctx.check(
        "canister pack pitches upward on elevation axis",
        pack_front_rest is not None
        and pack_front_raised is not None
        and pack_front_raised[1][2] > pack_front_rest[1][2] + 0.45,
        details=f"rest={pack_front_rest}, raised={pack_front_raised}",
    )
    ctx.check(
        "turntable slews the launcher around the trailer",
        turret_rest is not None
        and turret_turned is not None
        and abs(((turret_turned[0][1] + turret_turned[1][1]) * 0.5) - ((turret_rest[0][1] + turret_rest[1][1]) * 0.5)) > 0.30,
        details=f"rest={turret_rest}, turned={turret_turned}",
    )
    ctx.check(
        "service panel swings outward when opened",
        panel_rest is not None and panel_open is not None and panel_open[1][1] > panel_rest[1][1] + 0.18,
        details=f"rest={panel_rest}, open={panel_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
