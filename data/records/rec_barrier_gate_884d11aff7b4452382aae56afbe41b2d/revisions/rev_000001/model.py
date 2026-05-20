from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="railway_level_crossing_gate")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.62, 1.0))
    post_paint = model.material("post_paint", rgba=(0.18, 0.20, 0.21, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.31, 0.33, 0.36, 1.0))
    barrier_red = model.material("barrier_red", rgba=(0.70, 0.10, 0.11, 1.0))
    barrier_white = model.material("barrier_white", rgba=(0.95, 0.96, 0.94, 1.0))

    axis_x = 0.040
    lower_hinge_z = 0.380
    upper_hinge_z = 1.420
    hinge_spacing = upper_hinge_z - lower_hinge_z

    gate_depth = 0.060
    hinge_stile_x = 0.069
    latch_stile_x = 3.420
    rail_center_x = (hinge_stile_x + latch_stile_x) * 0.5
    rail_length = latch_stile_x - hinge_stile_x
    frame_bottom_z = -0.120
    frame_top_z = 1.170
    frame_center_z = (frame_bottom_z + frame_top_z) * 0.5
    frame_height = frame_top_z - frame_bottom_z
    top_rail_z = 1.120
    bottom_rail_z = -0.070
    mid_rail_z = 0.460
    face_center_z = 0.525

    def add_visual_box(
        part,
        *,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        name: str,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def diagonal_box(
        x0: float,
        z0: float,
        x1: float,
        z1: float,
        *,
        depth: float,
        thickness: float,
    ) -> tuple[Box, Origin]:
        dx = x1 - x0
        dz = z1 - z0
        length = sqrt(dx * dx + dz * dz)
        angle = atan2(dz, dx)
        return (
            Box((length, depth, thickness)),
            Origin(
                xyz=((x0 + x1) * 0.5, 0.0, (z0 + z1) * 0.5),
                rpy=(0.0, -angle, 0.0),
            ),
        )

    post = model.part("gatehouse_post")
    add_visual_box(
        post,
        size=(0.72, 0.72, 0.18),
        xyz=(-0.11, 0.0, 0.09),
        material=concrete,
        name="post_plinth",
    )
    add_visual_box(
        post,
        size=(0.22, 0.22, 1.86),
        xyz=(-0.11, 0.0, 0.93),
        material=post_paint,
        name="post_shaft",
    )
    add_visual_box(
        post,
        size=(0.26, 0.26, 0.06),
        xyz=(-0.11, 0.0, 1.89),
        material=post_paint,
        name="post_cap",
    )
    add_visual_box(
        post,
        size=(0.016, 0.110, 0.160),
        xyz=(axis_x * 0.5 - 0.012, 0.0, upper_hinge_z),
        material=hinge_steel,
        name="upper_post_bracket",
    )
    add_visual_box(
        post,
        size=(0.016, 0.110, 0.160),
        xyz=(axis_x * 0.5 - 0.012, 0.0, lower_hinge_z),
        material=hinge_steel,
        name="lower_post_bracket",
    )
    post.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 1.95)),
        mass=220.0,
        origin=Origin(xyz=(-0.11, 0.0, 0.98)),
    )

    upper_pintle = model.part("upper_pintle")
    upper_pintle.visual(
        Cylinder(radius=0.024, length=0.110),
        material=hinge_steel,
        name="upper_gate_knuckle",
    )
    upper_pintle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.110),
        mass=3.0,
    )

    lower_pintle = model.part("lower_pintle")
    lower_pintle.visual(
        Cylinder(radius=0.024, length=0.110),
        material=hinge_steel,
        name="lower_gate_knuckle",
    )
    lower_pintle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.110),
        mass=3.0,
    )

    gate_panel = model.part("gate_panel")
    add_visual_box(
        gate_panel,
        size=(0.090, gate_depth, frame_height),
        xyz=(hinge_stile_x, 0.0, frame_center_z),
        material=barrier_red,
        name="panel_hinge_stile",
    )
    add_visual_box(
        gate_panel,
        size=(0.090, gate_depth, frame_height),
        xyz=(latch_stile_x, 0.0, frame_center_z),
        material=barrier_red,
        name="panel_latch_stile",
    )
    add_visual_box(
        gate_panel,
        size=(rail_length, gate_depth, 0.100),
        xyz=(rail_center_x, 0.0, top_rail_z),
        material=barrier_red,
        name="panel_top_rail",
    )
    add_visual_box(
        gate_panel,
        size=(rail_length, gate_depth, 0.100),
        xyz=(rail_center_x, 0.0, bottom_rail_z),
        material=barrier_red,
        name="panel_bottom_rail",
    )
    add_visual_box(
        gate_panel,
        size=(rail_length, 0.050, 0.120),
        xyz=(rail_center_x, 0.0, mid_rail_z),
        material=barrier_red,
        name="panel_mid_rail",
    )
    add_visual_box(
        gate_panel,
        size=(0.190, 0.040, 0.080),
        xyz=(0.119, 0.0, 0.000),
        material=barrier_red,
        name="lower_hinge_strap",
    )
    add_visual_box(
        gate_panel,
        size=(0.190, 0.040, 0.080),
        xyz=(0.119, 0.0, hinge_spacing),
        material=barrier_red,
        name="upper_hinge_strap",
    )
    brace_geom, brace_origin = diagonal_box(
        0.155,
        -0.060,
        3.360,
        1.040,
        depth=0.038,
        thickness=0.085,
    )
    gate_panel.visual(
        brace_geom,
        origin=brace_origin,
        material=barrier_red,
        name="panel_diagonal_brace",
    )
    add_visual_box(
        gate_panel,
        size=(3.120, 0.014, 0.980),
        xyz=(1.755, 0.0, face_center_z),
        material=barrier_white,
        name="panel_face",
    )
    for index, center_x in enumerate((0.86, 1.56, 2.26, 2.96)):
        add_visual_box(
            gate_panel,
            size=(0.120, 0.018, 0.960),
            xyz=(center_x, 0.0, face_center_z),
            material=barrier_red,
            name=f"warning_band_{index}",
        )
    gate_panel.inertial = Inertial.from_geometry(
        Box((3.55, 0.08, 1.50)),
        mass=95.0,
        origin=Origin(xyz=(1.79, 0.0, 0.44)),
    )

    model.articulation(
        "post_to_upper_pintle",
        ArticulationType.REVOLUTE,
        parent=post,
        child=upper_pintle,
        origin=Origin(xyz=(axis_x, 0.0, upper_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.6,
            lower=0.0,
            upper=0.75,
        ),
    )
    model.articulation(
        "upper_to_lower_pintle",
        ArticulationType.REVOLUTE,
        parent=upper_pintle,
        child=lower_pintle,
        origin=Origin(xyz=(0.0, 0.0, -hinge_spacing)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.6,
            lower=0.0,
            upper=0.75,
        ),
    )
    model.articulation(
        "lower_pintle_to_gate",
        ArticulationType.FIXED,
        parent=lower_pintle,
        child=gate_panel,
        origin=Origin(),
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

    post = object_model.get_part("gatehouse_post")
    upper_pintle = object_model.get_part("upper_pintle")
    lower_pintle = object_model.get_part("lower_pintle")
    gate_panel = object_model.get_part("gate_panel")

    upper_hinge = object_model.get_articulation("post_to_upper_pintle")
    lower_hinge = object_model.get_articulation("upper_to_lower_pintle")

    ctx.check(
        "two coaxial revolute pintles drive the gate",
        upper_hinge.articulation_type == ArticulationType.REVOLUTE
        and lower_hinge.articulation_type == ArticulationType.REVOLUTE
        and upper_hinge.axis == (0.0, 0.0, 1.0)
        and lower_hinge.axis == (0.0, 0.0, 1.0)
        and abs(upper_hinge.origin.xyz[0] - 0.040) < 1e-9
        and abs(upper_hinge.origin.xyz[1]) < 1e-9
        and abs(lower_hinge.origin.xyz[0]) < 1e-9
        and abs(lower_hinge.origin.xyz[1]) < 1e-9
        and abs(lower_hinge.origin.xyz[2] + 1.040) < 1e-9,
        details=(
            f"upper_origin={upper_hinge.origin.xyz}, upper_axis={upper_hinge.axis}, "
            f"lower_origin={lower_hinge.origin.xyz}, lower_axis={lower_hinge.axis}"
        ),
    )

    with ctx.pose({upper_hinge: 0.0, lower_hinge: 0.0}):
        ctx.expect_contact(
            upper_pintle,
            post,
            elem_a="upper_gate_knuckle",
            elem_b="upper_post_bracket",
            contact_tol=0.001,
            name="upper pintle seats against the upper post bracket",
        )
        ctx.expect_contact(
            lower_pintle,
            post,
            elem_a="lower_gate_knuckle",
            elem_b="lower_post_bracket",
            contact_tol=0.001,
            name="lower pintle seats against the lower post bracket",
        )
        ctx.expect_contact(
            gate_panel,
            upper_pintle,
            elem_a="upper_hinge_strap",
            elem_b="upper_gate_knuckle",
            contact_tol=0.001,
            name="upper hinge strap meets the upper pintle knuckle",
        )
        ctx.expect_contact(
            gate_panel,
            lower_pintle,
            elem_a="lower_hinge_strap",
            elem_b="lower_gate_knuckle",
            contact_tol=0.001,
            name="lower hinge strap meets the lower pintle knuckle",
        )
        ctx.expect_gap(
            gate_panel,
            post,
            axis="x",
            min_gap=0.060,
            max_gap=0.075,
            positive_elem="panel_hinge_stile",
            negative_elem="post_shaft",
            name="closed gate frame stands just proud of the post",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    closed_latch_center = aabb_center(
        ctx.part_element_world_aabb(gate_panel, elem="panel_latch_stile")
    )

    upper_limit = 0.75
    lower_limit = 0.75
    with ctx.pose({upper_hinge: upper_limit, lower_hinge: lower_limit}):
        ctx.expect_contact(
            gate_panel,
            upper_pintle,
            elem_a="upper_hinge_strap",
            elem_b="upper_gate_knuckle",
            contact_tol=0.001,
            name="upper hinge stays seated when the gate is open",
        )
        open_latch_center = aabb_center(
            ctx.part_element_world_aabb(gate_panel, elem="panel_latch_stile")
        )

    with ctx.pose({upper_hinge: upper_limit, lower_hinge: 0.0}):
        upper_only_latch_center = aabb_center(
            ctx.part_element_world_aabb(gate_panel, elem="panel_latch_stile")
        )

    with ctx.pose({upper_hinge: 0.0, lower_hinge: lower_limit}):
        lower_only_latch_center = aabb_center(
            ctx.part_element_world_aabb(gate_panel, elem="panel_latch_stile")
        )

    ctx.check(
        "gate swings the free end out beside the roadway",
        closed_latch_center is not None
        and open_latch_center is not None
        and open_latch_center[1] > 2.6
        and open_latch_center[0] < closed_latch_center[0] - 2.4,
        details=(
            f"closed_latch_center={closed_latch_center}, "
            f"open_latch_center={open_latch_center}"
        ),
    )
    ctx.check(
        "both revolute pintles contribute to the swing",
        open_latch_center is not None
        and upper_only_latch_center is not None
        and lower_only_latch_center is not None
        and open_latch_center[1] > upper_only_latch_center[1] + 0.7
        and open_latch_center[1] > lower_only_latch_center[1] + 0.7,
        details=(
            f"both_open={open_latch_center}, "
            f"upper_only={upper_only_latch_center}, "
            f"lower_only={lower_only_latch_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
