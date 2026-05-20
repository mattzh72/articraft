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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_pickup_tailgate")

    steel = model.material("steel", rgba=(0.24, 0.26, 0.29, 1.0))
    painted_panel = model.material("painted_panel", rgba=(0.20, 0.22, 0.25, 1.0))
    inner_trim = model.material("inner_trim", rgba=(0.15, 0.16, 0.17, 1.0))
    handle_plastic = model.material("handle_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    handle_rubber = model.material("handle_rubber", rgba=(0.04, 0.04, 0.04, 1.0))

    bed_width = 1.78
    bed_depth = 0.12
    bed_height = 0.16

    gate_width = 1.70
    gate_height = 0.58
    gate_thickness = 0.064

    recess_width = 0.248
    recess_height = 0.118
    recess_depth = 0.036
    recess_center_z = 0.485
    handle_pivot_z = 0.520

    bed_edge = model.part("bed_edge")
    bed_edge.visual(
        Box((bed_width, bed_depth, bed_height)),
        origin=Origin(xyz=(0.0, 0.0, bed_height * 0.5)),
        material=steel,
        name="bed_sill",
    )
    bed_edge.visual(
        Box((bed_width, 0.06, 0.024)),
        origin=Origin(xyz=(0.0, 0.018, bed_height + 0.012)),
        material=steel,
        name="bed_cap",
    )
    for side in (-1.0, 1.0):
        bed_edge.visual(
            Box((0.080, 0.040, 0.085)),
            origin=Origin(xyz=(side * 0.79, -0.020, -0.0025)),
            material=steel,
            name=f"hinge_bracket_{'left' if side < 0.0 else 'right'}",
        )
    bed_edge.inertial = Inertial.from_geometry(
        Box((bed_width, bed_depth, bed_height + 0.05)),
        mass=45.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((gate_width, 0.014, gate_height)),
        origin=Origin(xyz=(0.0, -0.057, gate_height * 0.5)),
        material=painted_panel,
        name="outer_skin",
    )
    tailgate.visual(
        Box((gate_width, 0.060, 0.058)),
        origin=Origin(xyz=(0.0, -0.030, 0.029)),
        material=inner_trim,
        name="lower_beam",
    )
    tailgate.visual(
        Box((0.090, 0.054, 0.470)),
        origin=Origin(xyz=(-(gate_width * 0.5) + 0.045, -0.027, 0.315)),
        material=inner_trim,
        name="left_side_frame",
    )
    tailgate.visual(
        Box((0.090, 0.054, 0.470)),
        origin=Origin(xyz=((gate_width * 0.5) - 0.045, -0.027, 0.315)),
        material=inner_trim,
        name="right_side_frame",
    )
    top_segment_width = (gate_width - recess_width) * 0.5
    tailgate.visual(
        Box((top_segment_width, 0.054, 0.086)),
        origin=Origin(xyz=(-(recess_width * 0.5 + top_segment_width * 0.5), -0.027, 0.537)),
        material=inner_trim,
        name="upper_left_frame",
    )
    tailgate.visual(
        Box((top_segment_width, 0.054, 0.086)),
        origin=Origin(xyz=((recess_width * 0.5 + top_segment_width * 0.5), -0.027, 0.537)),
        material=inner_trim,
        name="upper_right_frame",
    )
    tailgate.visual(
        Box((1.42, 0.042, 0.360)),
        origin=Origin(xyz=(0.0, -0.021, 0.235)),
        material=inner_trim,
        name="inner_brace",
    )
    tailgate.visual(
        Box((recess_width, 0.008, recess_height)),
        origin=Origin(xyz=(0.0, -0.052, recess_center_z)),
        material=inner_trim,
        name="handle_recess_back",
    )
    tailgate.visual(
        Box((0.012, recess_depth, recess_height)),
        origin=Origin(xyz=(-(recess_width * 0.5) + 0.006, -0.018, recess_center_z)),
        material=inner_trim,
        name="handle_recess_left_wall",
    )
    tailgate.visual(
        Box((0.012, recess_depth, recess_height)),
        origin=Origin(xyz=((recess_width * 0.5) - 0.006, -0.018, recess_center_z)),
        material=inner_trim,
        name="handle_recess_right_wall",
    )
    tailgate.visual(
        Box((recess_width, recess_depth, 0.012)),
        origin=Origin(xyz=(0.0, -0.018, recess_center_z + recess_height * 0.5 - 0.006)),
        material=inner_trim,
        name="handle_recess_top",
    )
    tailgate.visual(
        Box((recess_width, recess_depth, 0.020)),
        origin=Origin(xyz=(0.0, -0.018, 0.410)),
        material=inner_trim,
        name="handle_recess_bottom",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((gate_width, gate_thickness, gate_height)),
        mass=32.0,
        origin=Origin(xyz=(0.0, -gate_thickness * 0.5, gate_height * 0.5)),
    )

    grab_handle = model.part("grab_handle")
    handle_loop = mesh_from_geometry(
        wire_from_points(
            [
                (-0.088, -0.004, -0.010),
                (-0.088, -0.022, -0.086),
                (0.088, -0.022, -0.086),
                (0.088, -0.004, -0.010),
            ],
            radius=0.010,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.024,
            corner_segments=10,
        ),
        "grab_handle_loop",
    )
    grab_handle.visual(handle_loop, material=handle_plastic, name="handle_loop")
    grab_handle.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(-0.099, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=handle_plastic,
        name="left_pivot_pin",
    )
    grab_handle.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(0.099, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=handle_plastic,
        name="right_pivot_pin",
    )
    grab_handle.visual(
        Box((0.028, 0.018, 0.020)),
        origin=Origin(xyz=(-0.092, -0.004, -0.008)),
        material=handle_plastic,
        name="left_handle_lug",
    )
    grab_handle.visual(
        Box((0.028, 0.018, 0.020)),
        origin=Origin(xyz=(0.092, -0.004, -0.008)),
        material=handle_plastic,
        name="right_handle_lug",
    )
    grab_handle.visual(
        Cylinder(radius=0.013, length=0.120),
        origin=Origin(xyz=(0.0, -0.022, -0.086), rpy=(0.0, pi * 0.5, 0.0)),
        material=handle_rubber,
        name="handle_grip",
    )
    grab_handle.inertial = Inertial.from_geometry(
        Box((0.230, 0.050, 0.110)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.015, -0.045)),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_edge,
        child=tailgate,
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=1.0,
            lower=0.0,
            upper=pi * 0.5,
        ),
    )
    model.articulation(
        "tailgate_to_grab_handle",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=grab_handle,
        origin=Origin(xyz=(0.0, -0.004, handle_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
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
    bed_edge = object_model.get_part("bed_edge")
    tailgate = object_model.get_part("tailgate")
    grab_handle = object_model.get_part("grab_handle")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    handle_hinge = object_model.get_articulation("tailgate_to_grab_handle")

    ctx.check(
        "tailgate hinge uses lower lateral axis",
        tailgate_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={tailgate_hinge.axis}",
    )
    ctx.check(
        "grab handle uses side pivot axis",
        handle_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={handle_hinge.axis}",
    )

    with ctx.pose({tailgate_hinge: 0.0, handle_hinge: 0.0}):
        ctx.expect_gap(
            bed_edge,
            tailgate,
            axis="y",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem="bed_sill",
            negative_elem="lower_beam",
            name="closed tailgate sits against the lower bed edge",
        )
        ctx.expect_overlap(
            bed_edge,
            tailgate,
            axes="x",
            min_overlap=1.65,
            elem_a="bed_sill",
            elem_b="outer_skin",
            name="tailgate spans the width of the bed opening",
        )
        ctx.expect_within(
            grab_handle,
            tailgate,
            axes="x",
            margin=0.03,
            name="closed grab handle stays centered within the tailgate recess zone",
        )

        closed_tailgate_aabb = ctx.part_world_aabb(tailgate)
        closed_handle_aabb = ctx.part_world_aabb(grab_handle)

    with ctx.pose({tailgate_hinge: pi * 0.5, handle_hinge: 0.0}):
        open_tailgate_aabb = ctx.part_world_aabb(tailgate)

    with ctx.pose({tailgate_hinge: 0.0, handle_hinge: 1.0}):
        open_handle_aabb = ctx.part_world_aabb(grab_handle)

    ctx.check(
        "tailgate folds down from vertical to horizontal",
        closed_tailgate_aabb is not None
        and open_tailgate_aabb is not None
        and closed_tailgate_aabb[1][2] > 0.55
        and open_tailgate_aabb[1][2] < 0.09
        and open_tailgate_aabb[0][1] < closed_tailgate_aabb[0][1] - 0.30,
        details=f"closed={closed_tailgate_aabb}, open={open_tailgate_aabb}",
    )
    ctx.check(
        "grab handle swings outward from the recess",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][1] > closed_handle_aabb[1][1] + 0.05,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
