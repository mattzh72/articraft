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
    Sphere,
    TestContext,
    TestReport,
)


def _add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_sphere(part, name, radius, xyz, material):
    part.visual(
        Sphere(radius=radius),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_watch_winder_box")

    shell = model.material("shell", rgba=(0.16, 0.18, 0.18, 1.0))
    lid_shell = model.material("lid_shell", rgba=(0.19, 0.21, 0.20, 1.0))
    gasket = model.material("gasket", rgba=(0.06, 0.06, 0.06, 1.0))
    hardware = model.material("hardware", rgba=(0.74, 0.76, 0.78, 1.0))
    liner = model.material("liner", rgba=(0.29, 0.22, 0.16, 1.0))
    cradle_finish = model.material("cradle_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    cushion = model.material("cushion", rgba=(0.57, 0.46, 0.34, 1.0))

    base_w = 0.280
    base_d = 0.200
    base_h = 0.105
    wall_t = 0.009
    floor_t = 0.012
    gasket_t = 0.002
    gasket_w = 0.007

    inner_w = base_w - 2.0 * wall_t
    inner_d = base_d - 2.0 * wall_t
    axis_z = 0.069

    base = model.part("base")
    _add_box(base, "floor", (base_w, base_d, floor_t), (0.0, 0.0, floor_t / 2.0), shell)
    _add_box(base, "left_wall", (wall_t, base_d, base_h), (-(base_w - wall_t) / 2.0, 0.0, base_h / 2.0), shell)
    _add_box(base, "right_wall", (wall_t, base_d, base_h), ((base_w - wall_t) / 2.0, 0.0, base_h / 2.0), shell)
    _add_box(base, "front_wall", (inner_w, wall_t, base_h), (0.0, (base_d - wall_t) / 2.0, base_h / 2.0), shell)
    _add_box(base, "back_wall", (inner_w, wall_t, base_h), (0.0, -(base_d - wall_t) / 2.0, base_h / 2.0), shell)
    _add_box(
        base,
        "liner",
        (inner_w - 0.018, inner_d - 0.018, 0.004),
        (0.0, 0.0, floor_t + 0.002),
        liner,
    )

    _add_box(
        base,
        "gasket_front",
        (inner_w - 0.018, gasket_w, gasket_t),
        (0.0, inner_d / 2.0 - gasket_w / 2.0, base_h + gasket_t / 2.0),
        gasket,
    )
    _add_box(
        base,
        "gasket_back",
        (inner_w - 0.018, gasket_w, gasket_t),
        (0.0, -(inner_d / 2.0 - gasket_w / 2.0), base_h + gasket_t / 2.0),
        gasket,
    )
    _add_box(
        base,
        "gasket_left",
        (gasket_w, inner_d - 2.0 * gasket_w, gasket_t),
        (-(inner_w / 2.0 - gasket_w / 2.0), 0.0, base_h + gasket_t / 2.0),
        gasket,
    )
    _add_box(
        base,
        "gasket_right",
        (gasket_w, inner_d - 2.0 * gasket_w, gasket_t),
        ((inner_w / 2.0 - gasket_w / 2.0), 0.0, base_h + gasket_t / 2.0),
        gasket,
    )

    pedestal_x = 0.110
    pedestal_size = (0.044, 0.050, 0.056)
    _add_box(base, "left_pedestal", pedestal_size, (-pedestal_x, 0.0, pedestal_size[2] / 2.0), shell)
    _add_box(base, "right_pedestal", pedestal_size, (pedestal_x, 0.0, pedestal_size[2] / 2.0), shell)
    _add_cylinder(
        base,
        "left_bearing_cap",
        radius=0.014,
        length=0.018,
        xyz=(-0.081, 0.0, axis_z),
        material=hardware,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_cylinder(
        base,
        "right_bearing_cap",
        radius=0.014,
        length=0.018,
        xyz=(0.081, 0.0, axis_z),
        material=hardware,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_box(base, "left_bearing_bridge", (0.020, 0.028, 0.020), (-0.092, 0.0, 0.065), shell)
    _add_box(base, "right_bearing_bridge", (0.020, 0.028, 0.020), (0.092, 0.0, 0.065), shell)

    hinge_axis_y = -(base_d / 2.0 + 0.004)
    hinge_axis_z = base_h + 0.014
    for side, x_center in (("left", -0.097), ("right", 0.097)):
        _add_box(
            base,
            f"hinge_leaf_{side}",
            (0.028, 0.010, 0.022),
            (x_center, hinge_axis_y + 0.003, 0.101),
            hardware,
        )
        _add_cylinder(
            base,
            f"hinge_barrel_{side}",
            radius=0.007,
            length=0.014,
            xyz=(x_center, hinge_axis_y, hinge_axis_z),
            material=hardware,
            rpy=(0.0, pi / 2.0, 0.0),
        )

    lid = model.part("lid")
    _add_box(lid, "top_panel", (0.300, 0.206, 0.010), (0.0, 0.117, -0.005), lid_shell)
    _add_box(lid, "front_skirt", (0.300, 0.008, 0.032), (0.0, 0.210, -0.026), lid_shell)
    _add_box(lid, "left_skirt", (0.008, 0.206, 0.032), (-0.146, 0.099, -0.026), lid_shell)
    _add_box(lid, "right_skirt", (0.008, 0.206, 0.032), (0.146, 0.099, -0.026), lid_shell)
    _add_box(lid, "hinge_mount_left", (0.010, 0.018, 0.010), (-0.083, 0.009, -0.005), hardware)
    _add_box(lid, "hinge_mount_right", (0.010, 0.018, 0.010), (0.083, 0.009, -0.005), hardware)
    _add_cylinder(
        lid,
        "hinge_pin_left",
        radius=0.007,
        length=0.014,
        xyz=(-0.083, 0.0, 0.0),
        material=hardware,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_cylinder(
        lid,
        "hinge_pin_right",
        radius=0.007,
        length=0.014,
        xyz=(0.083, 0.0, 0.0),
        material=hardware,
        rpy=(0.0, pi / 2.0, 0.0),
    )

    _add_box(lid, "rain_lip_front", (0.230, 0.006, 0.018), (0.0, 0.182, -0.019), lid_shell)
    _add_box(lid, "rain_lip_left", (0.006, 0.160, 0.018), (-0.120, 0.104, -0.019), lid_shell)
    _add_box(lid, "rain_lip_right", (0.006, 0.160, 0.018), (0.120, 0.104, -0.019), lid_shell)
    _add_box(lid, "rain_lip_back_left", (0.050, 0.006, 0.012), (-0.086, 0.028, -0.016), lid_shell)
    _add_box(lid, "rain_lip_back_right", (0.050, 0.006, 0.012), (0.086, 0.028, -0.016), lid_shell)

    _add_box(lid, "seal_front", (inner_w - 0.018, gasket_w, gasket_t), (0.0, 0.1915, -0.011), gasket)
    _add_box(lid, "seal_left", (gasket_w, inner_d - 2.0 * gasket_w, gasket_t), (-0.1275, 0.104, -0.011), gasket)
    _add_box(lid, "seal_right", (gasket_w, inner_d - 2.0 * gasket_w, gasket_t), (0.1275, 0.104, -0.011), gasket)
    _add_box(lid, "seal_back_left", (0.090, gasket_w, gasket_t), (-0.076, 0.0165, -0.011), gasket)
    _add_box(lid, "seal_back_right", (0.090, gasket_w, gasket_t), (0.076, 0.0165, -0.011), gasket)

    cradle = model.part("cradle")
    _add_cylinder(
        cradle,
        "drum",
        radius=0.038,
        length=0.080,
        xyz=(0.0, 0.0, 0.0),
        material=cradle_finish,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_cylinder(
        cradle,
        "left_collar",
        radius=0.042,
        length=0.006,
        xyz=(-0.043, 0.0, 0.0),
        material=cradle_finish,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_cylinder(
        cradle,
        "right_collar",
        radius=0.042,
        length=0.006,
        xyz=(0.043, 0.0, 0.0),
        material=cradle_finish,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_box(cradle, "cushion", (0.056, 0.028, 0.050), (0.0, 0.023, 0.0), cushion)
    _add_cylinder(
        cradle,
        "left_axle",
        radius=0.005,
        length=0.020,
        xyz=(-0.050, 0.0, 0.0),
        material=hardware,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_cylinder(
        cradle,
        "right_axle",
        radius=0.005,
        length=0.020,
        xyz=(0.050, 0.0, 0.0),
        material=hardware,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_sphere(cradle, "left_pivot", 0.006, (-0.066, 0.0, 0.0), hardware)
    _add_sphere(cradle, "right_pivot", 0.006, (0.066, 0.0, 0.0), hardware)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.28),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")

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

    ctx.expect_contact(lid, base, elem_a="seal_front", elem_b="gasket_front", name="front_seal_contacts_gasket")
    ctx.expect_contact(lid, base, elem_a="seal_left", elem_b="gasket_left", name="left_seal_contacts_gasket")
    ctx.expect_contact(lid, base, elem_a="seal_back_left", elem_b="gasket_back", name="rear_seal_contacts_gasket")
    ctx.expect_contact(lid, base, elem_a="hinge_pin_left", elem_b="hinge_barrel_left", name="left_hinge_support_contact")
    ctx.expect_contact(lid, base, elem_a="hinge_pin_right", elem_b="hinge_barrel_right", name="right_hinge_support_contact")

    ctx.expect_contact(cradle, base, elem_a="left_pivot", elem_b="left_bearing_cap", name="left_pivot_is_supported")
    ctx.expect_contact(cradle, base, elem_a="right_pivot", elem_b="right_bearing_cap", name="right_pivot_is_supported")
    ctx.expect_gap(
        cradle,
        base,
        axis="z",
        positive_elem="drum",
        negative_elem="liner",
        min_gap=0.015,
        name="drum_clears_liner",
    )
    ctx.expect_within(cradle, base, axes="xy", margin=0.0, name="cradle_stays_within_box_footprint")

    with ctx.pose({lid_hinge: 1.15}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="gasket_front",
            min_gap=0.10,
            name="lid_front_edge_lifts_clear_when_open",
        )

    with ctx.pose({cradle_spin: pi / 2.0}):
        ctx.expect_contact(
            cradle,
            base,
            elem_a="right_pivot",
            elem_b="right_bearing_cap",
            name="right_pivot_stays_supported_while_spinning",
        )
        ctx.expect_gap(
            cradle,
            base,
            axis="z",
            positive_elem="cushion",
            negative_elem="liner",
            min_gap=0.008,
            name="cushion_clears_liner_during_rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
