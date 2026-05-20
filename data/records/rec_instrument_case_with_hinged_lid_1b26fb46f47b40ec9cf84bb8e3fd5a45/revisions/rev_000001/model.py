from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    sample_catmull_rom_spline_2d,
)


LOWER_HEIGHT = 0.078
FLOOR_THICKNESS = 0.018
LID_HEIGHT = 0.055
LID_TOP_SKIN = 0.015
HINGE_Y = 0.282
HINGE_Z = LOWER_HEIGHT + 0.006
LID_Z_OFFSET = -0.004


def _outer_profile() -> list[tuple[float, float]]:
    """Smooth fitted guitar-case footprint in the local XY plane."""
    top_controls = [
        (-0.625, 0.060),
        (-0.585, 0.120),
        (-0.455, 0.124),
        (-0.315, 0.086),
        (-0.075, 0.092),
        (0.095, 0.172),
        (0.230, 0.194),
        (0.345, 0.146),
        (0.475, 0.246),
        (0.610, 0.228),
        (0.650, 0.150),
    ]
    top = sample_catmull_rom_spline_2d(
        top_controls, samples_per_segment=10, closed=False
    )
    bottom = [(x, -y) for x, y in reversed(top)]
    return top + bottom


def _scale_profile(
    profile: list[tuple[float, float]], sx: float, sy: float
) -> list[tuple[float, float]]:
    return [(x * sx, y * sy) for x, y in profile]


def _profile_solid(profile: list[tuple[float, float]], height: float) -> cq.Workplane:
    return cq.Workplane("XY").polyline(profile).close().extrude(height)


def _lower_shell_cq() -> cq.Workplane:
    outer = _profile_solid(_outer_profile(), LOWER_HEIGHT)
    inner = _profile_solid(_scale_profile(_outer_profile(), 0.930, 0.760), LOWER_HEIGHT)
    inner = inner.translate((0.0, 0.0, FLOOR_THICKNESS))
    return outer.cut(inner)


def _lid_shell_cq() -> cq.Workplane:
    outer = _profile_solid(_outer_profile(), LID_HEIGHT)
    cavity = _profile_solid(
        _scale_profile(_outer_profile(), 0.920, 0.745),
        LID_HEIGHT - LID_TOP_SKIN + 0.015,
    ).translate((0.0, 0.0, -0.010))
    return outer.cut(cavity)


def _foam_insert_cq() -> cq.Workplane:
    base_profile = _scale_profile(_outer_profile(), 0.875, 0.675)
    foam = _profile_solid(base_profile, 0.012)
    # Raised pads and a neck cradle are molded into the plush lining.
    neck_cradle = cq.Workplane("XY").box(0.430, 0.048, 0.022).translate(
        (-0.260, 0.0, 0.023)
    )
    upper_pad = cq.Workplane("XY").ellipse(0.130, 0.055).extrude(0.014).translate(
        (0.135, 0.0, 0.012)
    )
    lower_pad = cq.Workplane("XY").ellipse(0.185, 0.082).extrude(0.018).translate(
        (0.465, 0.0, 0.012)
    )
    head_pad = cq.Workplane("XY").box(0.090, 0.070, 0.014).translate(
        (-0.545, 0.0, 0.019)
    )
    return foam.union(neck_cradle).union(upper_pad).union(lower_pad).union(head_pad)


def _lid_lining_cq() -> cq.Workplane:
    return _profile_solid(_scale_profile(_outer_profile(), 0.865, 0.650), 0.007)


def _front_y_for_latch(x: float) -> float:
    # Latches sit on the broad lower-bout front of the fitted case, just outside
    # the global shell envelope.
    return -0.265


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fitted_guitar_instrument_case")

    shell_mat = model.material("black_rigid_shell", rgba=(0.025, 0.027, 0.030, 1.0))
    subtle_shell_mat = model.material(
        "raised_molded_ribs", rgba=(0.040, 0.043, 0.047, 1.0)
    )
    rubber_mat = model.material("matte_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.70, 0.68, 0.62, 1.0))
    lining_mat = model.material("blue_plush_lining", rgba=(0.045, 0.095, 0.180, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        mesh_from_cadquery(_lower_shell_cq(), "lower_shell"),
        material=shell_mat,
        name="lower_shell",
    )

    # Molded handle bosses and front latch keepers are embedded into the lower
    # half so the hardware reads as bolted to a protective shell.
    handle_x = 0.170
    handle_y = -0.224
    for i, dx in enumerate((-0.095, 0.095)):
        lower.visual(
            Box((0.040, 0.075, 0.020)),
            origin=Origin(xyz=(handle_x + dx, handle_y + 0.037, 0.038)),
            material=subtle_shell_mat,
            name=f"handle_boss_{i}",
        )
        lower.visual(
            Box((0.032, 0.010, 0.036)),
            origin=Origin(xyz=(handle_x + dx, handle_y + 0.014, 0.055)),
            material=metal_mat,
            name=f"handle_plate_{i}",
        )

    latch_xs = [0.455, 0.535, 0.615]
    for i, x in enumerate(latch_xs):
        y = _front_y_for_latch(x)
        lower.visual(
            Box((0.078, 0.040, 0.034)),
            origin=Origin(xyz=(x, y + 0.004, 0.044)),
            material=metal_mat,
            name=f"keeper_{i}",
        )
        lower.visual(
            Cylinder(radius=0.006, length=0.055),
            origin=Origin(xyz=(x, y - 0.011, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"keeper_pin_{i}",
        )

    # Small rubber feet keep the rigid shell off the ground.
    for i, (x, y) in enumerate(
        [(-0.520, 0.070), (-0.035, 0.095), (0.410, 0.155), (0.560, -0.120)]
    ):
        lower.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(x, y, -0.005)),
            material=rubber_mat,
            name=f"foot_{i}",
        )

    lower.visual(
        mesh_from_cadquery(_foam_insert_cq(), "fitted_foam_insert"),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS)),
        material=lining_mat,
        name="fitted_foam",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_cq(), "lid_shell"),
        origin=Origin(xyz=(0.0, -HINGE_Y, LID_Z_OFFSET)),
        material=shell_mat,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_lid_lining_cq(), "lid_lining"),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.032)),
        material=lining_mat,
        name="lid_lining",
    )
    # Raised molded ribs and a small blank name plate on the outside of the lid.
    for i, (x, y, sx, sy) in enumerate(
        [
            (-0.340, 0.000, 0.360, 0.020),
            (0.155, 0.070, 0.260, 0.018),
            (0.445, 0.115, 0.220, 0.018),
            (0.445, -0.115, 0.220, 0.018),
        ]
    ):
        lid.visual(
            Box((sx, sy, 0.006)),
            origin=Origin(xyz=(x, y - HINGE_Y, LID_Z_OFFSET + LID_HEIGHT + 0.002)),
            material=subtle_shell_mat,
            name=f"top_rib_{i}",
        )
    lid.visual(
        Box((0.110, 0.044, 0.004)),
        origin=Origin(xyz=(0.240, -HINGE_Y, LID_Z_OFFSET + LID_HEIGHT + 0.002)),
        material=metal_mat,
        name="name_plate",
    )

    # Exposed hinge knuckles are split between the lower shell and lid so the
    # visible leaves follow the moving half.
    for i, x in enumerate((0.160, 0.420, 0.560)):
        lower.visual(
            Box((0.110, 0.100, 0.008)),
            origin=Origin(xyz=(x, HINGE_Y - 0.060, HINGE_Z - 0.008)),
            material=metal_mat,
            name=f"lower_hinge_leaf_{i}",
        )
        lower.visual(
            Cylinder(radius=0.007, length=0.066),
            origin=Origin(xyz=(x - 0.034, HINGE_Y - 0.008, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"lower_knuckle_{i}",
        )
        lid.visual(
            Box((0.065, 0.130, 0.008)),
            origin=Origin(xyz=(x + 0.032, -0.055, 0.0)),
            material=metal_mat,
            name=f"lid_hinge_leaf_{i}",
        )
        lid.visual(
            Cylinder(radius=0.0064, length=0.058),
            origin=Origin(xyz=(x + 0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"lid_knuckle_{i}",
        )

    lid_joint = model.articulation(
        "lower_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.85),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.210),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="pivot_pin",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(xyz=(0.0, -0.055, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_mat,
        name="grip",
    )
    for i, x in enumerate((-0.082, 0.082)):
        handle.visual(
            Box((0.012, 0.100, 0.070)),
            origin=Origin(xyz=(x, -0.050, -0.030)),
            material=metal_mat,
            name=f"strap_{i}",
        )
    handle_joint = model.articulation(
        "lower_to_handle",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=handle,
        origin=Origin(xyz=(handle_x, handle_y, 0.055)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    for i, x in enumerate(latch_xs):
        y = _front_y_for_latch(x)
        pivot_world_z = HINGE_Z + 0.014
        pivot_world_y = y - 0.020
        lid.visual(
            Box((0.078, 0.040, 0.028)),
            origin=Origin(xyz=(x, y - HINGE_Y + 0.0055, 0.014)),
            material=metal_mat,
            name=f"latch_base_{i}",
        )

        latch = model.part(f"latch_{i}")
        latch.visual(
            Box((0.054, 0.006, 0.064)),
            origin=Origin(xyz=(0.0, -0.004, -0.032)),
            material=metal_mat,
            name="lever",
        )
        latch.visual(
            Box((0.050, 0.014, 0.010)),
            origin=Origin(xyz=(0.0, -0.009, -0.064)),
            material=metal_mat,
            name="hook_lip",
        )
        latch.visual(
            Cylinder(radius=0.0055, length=0.066),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="pivot_barrel",
        )
        model.articulation(
            f"lid_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(
                xyz=(
                    x,
                    pivot_world_y - HINGE_Y,
                    pivot_world_z - HINGE_Z,
                )
            ),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0, velocity=3.0, lower=0.0, upper=1.15
            ),
        )

    # Store the main joint names for simple prompt-specific tests.
    model.meta["main_lid_joint"] = lid_joint.name
    model.meta["handle_joint"] = handle_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    latch = object_model.get_part("latch_1")

    lid_joint = object_model.get_articulation("lower_to_lid")
    handle_joint = object_model.get_articulation("lower_to_handle")
    latch_joint = object_model.get_articulation("lid_to_latch_1")

    ctx.expect_gap(
        lid,
        lower,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="lower_shell",
        min_gap=0.001,
        max_gap=0.004,
        name="closed lid has a thin protective seam gap",
    )
    ctx.expect_overlap(
        lid,
        lower,
        axes="xy",
        elem_a="lid_shell",
        elem_b="lower_shell",
        min_overlap=0.40,
        name="lid follows the fitted case footprint",
    )
    ctx.expect_within(
        lower,
        lower,
        axes="xy",
        inner_elem="fitted_foam",
        outer_elem="lower_shell",
        margin=0.005,
        name="plush insert is contained inside the lower shell",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "hinged lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_joint: 1.05}):
        open_latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "center latch flips outward",
        rest_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[0][1] < rest_latch_aabb[0][1] - 0.015,
        details=f"rest={rest_latch_aabb}, open={open_latch_aabb}",
    )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.05}):
        raised_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "folding handle lifts outward",
        rest_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[0][1] < rest_handle_aabb[0][1] - 0.004
        and raised_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.025,
        details=f"rest={rest_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
