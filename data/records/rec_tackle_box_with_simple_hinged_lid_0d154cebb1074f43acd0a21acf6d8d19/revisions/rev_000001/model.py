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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_L = 0.42
BODY_D = 0.22
BODY_H = 0.14
BODY_WALL = 0.005
LID_H = 0.08


def _cylinder_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_box(part, name: str, size, xyz, *, material: str, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    name: str,
    *,
    radius: float,
    length: float,
    xyz,
    axis: str,
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_cylinder_rpy(axis)),
        material=material,
        name=name,
    )


def _add_end_service_hatch(part, *, prefix: str, x_sign: float) -> None:
    panel_x = x_sign * (BODY_L / 2.0 + 0.0015)
    hinge_x = x_sign * (BODY_L / 2.0 + 0.0035)
    bolt_x = x_sign * (BODY_L / 2.0 + 0.0055)

    _add_box(
        part,
        f"{prefix}_service_hatch",
        (0.003, 0.098, 0.070),
        (panel_x, 0.0, 0.080),
        material="adapter",
    )
    _add_box(
        part,
        f"{prefix}_service_hinge_strip",
        (0.005, 0.098, 0.010),
        (hinge_x, 0.0, 0.113),
        material="hardware_dark",
    )
    for i, (bolt_y, bolt_z) in enumerate(
        ((-0.038, 0.105), (0.038, 0.105), (-0.038, 0.055), (0.038, 0.055))
    ):
        _add_cylinder(
            part,
            f"{prefix}_service_bolt_{i}",
            radius=0.004,
            length=0.006,
            xyz=(bolt_x, bolt_y, bolt_z),
            axis="x",
            material="hardware",
        )


def _add_side_adapter(part, *, prefix: str, y_sign: float, x_center: float) -> None:
    plate_y = y_sign * (BODY_D / 2.0 + 0.0015)
    bolt_y = y_sign * (BODY_D / 2.0 + 0.0050)

    _add_box(
        part,
        f"{prefix}_adapter_plate",
        (0.090, 0.003, 0.042),
        (x_center, plate_y, 0.045),
        material="adapter",
    )
    for i, bolt_x in enumerate((x_center - 0.028, x_center + 0.028)):
        _add_cylinder(
            part,
            f"{prefix}_adapter_bolt_{i}",
            radius=0.004,
            length=0.004,
            xyz=(bolt_x, bolt_y, 0.045),
            axis="y",
            material="hardware",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_tackle_box")

    model.material("shell", rgba=(0.29, 0.36, 0.26, 1.0))
    model.material("shell_dark", rgba=(0.20, 0.24, 0.19, 1.0))
    model.material("hardware", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("hardware_dark", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("adapter", rgba=(0.52, 0.54, 0.56, 1.0))
    model.material("gasket", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")
    lid = model.part("lid")
    latch = model.part("front_latch")

    _add_box(
        body,
        "floor",
        (BODY_L - 0.010, BODY_D - 0.010, BODY_WALL),
        (0.0, 0.0, BODY_WALL / 2.0),
        material="shell",
    )
    _add_box(
        body,
        "front_wall",
        (BODY_L, BODY_WALL, BODY_H),
        (0.0, BODY_D / 2.0 - BODY_WALL / 2.0, BODY_H / 2.0),
        material="shell",
    )
    _add_box(
        body,
        "back_wall",
        (BODY_L, BODY_WALL, BODY_H),
        (0.0, -BODY_D / 2.0 + BODY_WALL / 2.0, BODY_H / 2.0),
        material="shell",
    )
    _add_box(
        body,
        "left_wall",
        (BODY_WALL, BODY_D - 0.010, BODY_H),
        (-BODY_L / 2.0 + BODY_WALL / 2.0, 0.0, BODY_H / 2.0),
        material="shell",
    )
    _add_box(
        body,
        "right_wall",
        (BODY_WALL, BODY_D - 0.010, BODY_H),
        (BODY_L / 2.0 - BODY_WALL / 2.0, 0.0, BODY_H / 2.0),
        material="shell",
    )

    _add_box(body, "front_rim", (BODY_L - 0.030, 0.016, 0.010), (0.0, BODY_D / 2.0 - 0.012, BODY_H - 0.005), material="shell_dark")
    _add_box(body, "left_rim", (0.016, BODY_D - 0.034, 0.010), (-BODY_L / 2.0 + 0.012, 0.0, BODY_H - 0.005), material="shell_dark")
    _add_box(body, "right_rim", (0.016, BODY_D - 0.034, 0.010), (BODY_L / 2.0 - 0.012, 0.0, BODY_H - 0.005), material="shell_dark")

    for prefix, sx, sy in (
        ("front_left", -1.0, 1.0),
        ("front_right", 1.0, 1.0),
        ("rear_left", -1.0, -1.0),
        ("rear_right", 1.0, -1.0),
    ):
        _add_box(
            body,
            f"{prefix}_corner_reinforcement",
            (0.018, 0.018, 0.062),
            (sx * (BODY_L / 2.0 - 0.009), sy * (BODY_D / 2.0 - 0.009), 0.031),
            material="shell_dark",
        )

    _add_box(body, "hinge_doubler", (0.300, 0.003, 0.032), (0.0, -BODY_D / 2.0 - 0.0015, BODY_H - 0.016), material="adapter")
    _add_box(body, "keeper_plate", (0.090, 0.003, 0.040), (0.0, BODY_D / 2.0 + 0.0015, 0.154), material="adapter")
    _add_box(body, "keeper_leg_left", (0.010, 0.014, 0.022), (-0.018, BODY_D / 2.0 + 0.007, 0.149), material="hardware_dark")
    _add_box(body, "keeper_leg_right", (0.010, 0.014, 0.022), (0.018, BODY_D / 2.0 + 0.007, 0.149), material="hardware_dark")
    _add_box(body, "keeper_bar", (0.050, 0.008, 0.010), (0.0, BODY_D / 2.0 + 0.012, 0.157), material="hardware")

    _add_end_service_hatch(body, prefix="left", x_sign=-1.0)
    _add_end_service_hatch(body, prefix="right", x_sign=1.0)
    _add_side_adapter(body, prefix="front_left", y_sign=1.0, x_center=-0.118)
    _add_side_adapter(body, prefix="front_right", y_sign=1.0, x_center=0.118)
    _add_side_adapter(body, prefix="rear_left", y_sign=-1.0, x_center=-0.118)
    _add_side_adapter(body, prefix="rear_right", y_sign=-1.0, x_center=0.118)

    for i, x in enumerate((-0.120, 0.120)):
        _add_cylinder(
            body,
            f"body_hinge_knuckle_{i}",
            radius=0.006,
            length=0.090,
            xyz=(x, -BODY_D / 2.0 - 0.006, BODY_H),
            axis="x",
            material="hardware_dark",
        )
        _add_box(
            body,
            f"body_hinge_bracket_{i}",
            (0.090, 0.012, 0.018),
            (x, -BODY_D / 2.0, BODY_H - 0.009),
            material="hardware_dark",
        )

    lid_outer_l = BODY_L + 0.008
    lid_outer_d = BODY_D + 0.004

    _add_box(lid, "lid_top", (lid_outer_l, lid_outer_d, BODY_WALL), (0.0, lid_outer_d / 2.0, LID_H - BODY_WALL / 2.0), material="shell")
    _add_box(
        lid,
        "lid_back_wall",
        (0.110, BODY_WALL, LID_H - BODY_WALL),
        (0.0, BODY_WALL / 2.0, (LID_H - BODY_WALL) / 2.0),
        material="shell",
    )
    _add_box(
        lid,
        "lid_back_wall_left_outer",
        (0.042, BODY_WALL, LID_H - BODY_WALL),
        (-0.188, BODY_WALL / 2.0, (LID_H - BODY_WALL) / 2.0),
        material="shell",
    )
    _add_box(
        lid,
        "lid_back_wall_right_outer",
        (0.042, BODY_WALL, LID_H - BODY_WALL),
        (0.188, BODY_WALL / 2.0, (LID_H - BODY_WALL) / 2.0),
        material="shell",
    )
    _add_box(lid, "lid_front_wall", (lid_outer_l, BODY_WALL, LID_H), (0.0, lid_outer_d - BODY_WALL / 2.0, LID_H / 2.0), material="shell")
    _add_box(lid, "lid_left_wall", (BODY_WALL, lid_outer_d - 0.010, LID_H), (-lid_outer_l / 2.0 + BODY_WALL / 2.0, (lid_outer_d - 0.010) / 2.0, LID_H / 2.0), material="shell")
    _add_box(lid, "lid_right_wall", (BODY_WALL, lid_outer_d - 0.010, LID_H), (lid_outer_l / 2.0 - BODY_WALL / 2.0, (lid_outer_d - 0.010) / 2.0, LID_H / 2.0), material="shell")

    _add_box(lid, "lid_top_rib_left", (0.120, 0.016, 0.010), (-0.095, 0.105, LID_H - 0.0005), material="shell_dark")
    _add_box(lid, "lid_top_rib_right", (0.120, 0.016, 0.010), (0.095, 0.105, LID_H - 0.0005), material="shell_dark")
    _add_box(lid, "lid_front_doubler", (0.120, 0.003, 0.050), (0.0, lid_outer_d + 0.0015, 0.056), material="adapter")
    _add_box(lid, "latch_mount_pad", (0.060, 0.003, 0.018), (0.0, lid_outer_d + 0.0015, 0.064), material="hardware_dark")
    _add_box(lid, "lid_gasket_strip", (BODY_L - 0.030, 0.008, 0.006), (0.0, lid_outer_d - 0.016, LID_H - BODY_WALL - 0.003), material="gasket")

    _add_cylinder(
        lid,
        "lid_center_knuckle",
        radius=0.006,
        length=0.110,
        xyz=(0.0, -0.006, 0.0),
        axis="x",
        material="hardware",
    )
    _add_box(lid, "lid_hinge_bracket", (0.110, 0.012, 0.018), (0.0, 0.000, 0.009), material="hardware_dark")

    _add_cylinder(
        latch,
        "pivot_barrel",
        radius=0.004,
        length=0.050,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material="hardware",
    )
    _add_box(latch, "latch_plate", (0.052, 0.004, 0.048), (0.0, 0.002, -0.026), material="hardware")
    _add_box(latch, "hook_lip", (0.032, 0.010, 0.010), (0.0, 0.001, -0.057), material="hardware_dark")
    _add_box(latch, "hook_stem", (0.020, 0.006, 0.004), (0.0, 0.002, -0.051), material="hardware_dark")
    _add_box(latch, "latch_reinforcement", (0.028, 0.006, 0.016), (0.0, 0.005, -0.010), material="hardware_dark")

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 - 0.006, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, lid_outer_d + 0.003, 0.064)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("front_latch")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_hinge = object_model.get_articulation("lid_to_latch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "core parts present",
        all(part is not None for part in (body, lid, latch)),
        "body, lid, and latch should all exist as the supported tackle-box mechanism.",
    )

    with ctx.pose({lid_hinge: 0.0, latch_hinge: 0.0}):
        ctx.expect_contact(body, lid, name="lid seats on body in closed pose")
        ctx.expect_contact(
            latch,
            lid,
            elem_a="latch_plate",
            elem_b="latch_mount_pad",
            name="latch remains mounted to lid hardware",
        )
        ctx.expect_gap(
            body,
            latch,
            axis="y",
            positive_elem="keeper_bar",
            negative_elem="hook_lip",
            min_gap=0.0,
            max_gap=0.004,
            name="closed latch lines up with body keeper",
        )
        ctx.expect_gap(
            body,
            latch,
            axis="z",
            positive_elem="keeper_bar",
            negative_elem="hook_lip",
            max_gap=0.002,
            max_penetration=0.0005,
            name="closed latch hook sits just under keeper bar",
        )
        ctx.expect_overlap(
            latch,
            body,
            axes="x",
            min_overlap=0.020,
            elem_a="hook_lip",
            elem_b="keeper_bar",
            name="closed latch hook spans keeper width",
        )

    with ctx.pose({lid_hinge: 1.10, latch_hinge: 1.00}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front_wall",
            min_gap=0.050,
            name="opened lid lifts front wall clear of shell",
        )
        ctx.expect_gap(
            body,
            latch,
            axis="y",
            positive_elem="keeper_bar",
            negative_elem="hook_lip",
            min_gap=0.080,
            name="opened latch swings clear of keeper",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
