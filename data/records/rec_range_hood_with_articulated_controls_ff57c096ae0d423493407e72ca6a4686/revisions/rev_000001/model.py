from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def _add_filter_cartridge(
    model: ArticulatedObject,
    *,
    name: str,
    body_part,
    hinge_origin: tuple[float, float, float],
    axis: tuple[float, float, float],
    inward_sign: float,
    filter_material,
    mesh_material,
) -> None:
    filter_width = 0.232
    filter_depth = 0.232
    frame_thickness = 0.014

    cartridge = model.part(name)
    cartridge.visual(
        Cylinder(radius=0.006, length=filter_depth - 0.012),
        origin=Origin(
            xyz=(inward_sign * 0.006, 0.0, -0.001),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=filter_material,
        name="hinge_barrel",
    )
    cartridge.visual(
        Box((0.014, filter_depth - 0.012, 0.006)),
        origin=Origin(xyz=(inward_sign * 0.007, 0.0, -0.003)),
        material=filter_material,
        name="hinge_leaf",
    )
    cartridge.visual(
        Box((0.020, filter_depth, frame_thickness)),
        origin=Origin(xyz=(inward_sign * 0.010, 0.0, -0.008)),
        material=filter_material,
        name="outer_rail",
    )
    cartridge.visual(
        Box((0.018, filter_depth, frame_thickness)),
        origin=Origin(xyz=(inward_sign * (filter_width - 0.009), 0.0, -0.008)),
        material=filter_material,
        name="inner_rail",
    )
    cartridge.visual(
        Box((filter_width, 0.018, frame_thickness)),
        origin=Origin(
            xyz=(inward_sign * (filter_width * 0.5), -(filter_depth * 0.5) + 0.009, -0.008)
        ),
        material=filter_material,
        name="front_rail",
    )
    cartridge.visual(
        Box((filter_width, 0.018, frame_thickness)),
        origin=Origin(
            xyz=(inward_sign * (filter_width * 0.5), (filter_depth * 0.5) - 0.009, -0.008)
        ),
        material=filter_material,
        name="rear_rail",
    )
    cartridge.visual(
        Box((filter_width - 0.028, filter_depth - 0.028, 0.004)),
        origin=Origin(xyz=(inward_sign * (filter_width * 0.5), 0.0, -0.008)),
        material=mesh_material,
        name="filter_mesh",
    )
    for index, fraction in enumerate((0.26, 0.50, 0.74)):
        cartridge.visual(
            Box((0.008, filter_depth - 0.036, 0.006)),
            origin=Origin(xyz=(inward_sign * (filter_width * fraction), 0.0, -0.007)),
            material=filter_material,
            name=f"mesh_rib_{index}",
        )
    cartridge.visual(
        Box((0.022, 0.010, 0.010)),
        origin=Origin(
            xyz=(inward_sign * (filter_width - 0.014), -(filter_depth * 0.5) + 0.012, -0.016)
        ),
        material=filter_material,
        name="pull_tab",
    )
    cartridge.inertial = Inertial.from_geometry(
        Box((filter_width, filter_depth, 0.024)),
        mass=0.75,
        origin=Origin(xyz=(inward_sign * (filter_width * 0.5), 0.0, -0.009)),
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.REVOLUTE,
        parent=body_part,
        child=cartridge,
        origin=Origin(xyz=hinge_origin),
        axis=axis,
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_hood_insert")

    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.58, 0.60, 0.62, 1.0))
    filter_mesh = model.material("filter_mesh_dark", rgba=(0.28, 0.30, 0.32, 1.0))
    collar_galv = model.material("collar_galv", rgba=(0.74, 0.76, 0.78, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_marker = model.material("knob_marker", rgba=(0.88, 0.20, 0.12, 1.0))

    body_width = 0.560
    body_depth = 0.310
    body_height = 0.185
    wall_thickness = 0.014
    top_thickness = 0.018
    opening_width = 0.4802
    opening_depth = 0.240
    lip_thickness = 0.016

    body = model.part("insert_body")
    body.visual(
        Box((body_width, body_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - (top_thickness * 0.5))),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height - top_thickness + 0.003)),
        origin=Origin(xyz=(-(body_width * 0.5) + (wall_thickness * 0.5), 0.0, 0.085)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height - top_thickness + 0.003)),
        origin=Origin(xyz=((body_width * 0.5) - (wall_thickness * 0.5), 0.0, 0.085)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((body_width - (2.0 * wall_thickness), wall_thickness, body_height - top_thickness + 0.003)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5) + (wall_thickness * 0.5), 0.085)),
        material=stainless,
        name="front_fascia",
    )
    body.visual(
        Box((body_width - (2.0 * wall_thickness), wall_thickness, body_height - top_thickness + 0.003)),
        origin=Origin(xyz=(0.0, (body_depth * 0.5) - (wall_thickness * 0.5), 0.085)),
        material=stainless,
        name="rear_wall",
    )
    body.visual(
        Box((opening_width, (body_depth - opening_depth) * 0.5, lip_thickness)),
        origin=Origin(
            xyz=(0.0, -(opening_depth * 0.5) - ((body_depth - opening_depth) * 0.25), lip_thickness * 0.5)
        ),
        material=brushed_dark,
        name="intake_front_lip",
    )
    body.visual(
        Box((opening_width, (body_depth - opening_depth) * 0.5, lip_thickness)),
        origin=Origin(
            xyz=(0.0, (opening_depth * 0.5) + ((body_depth - opening_depth) * 0.25), lip_thickness * 0.5)
        ),
        material=brushed_dark,
        name="intake_rear_lip",
    )
    body.visual(
        Box(((body_width - opening_width) * 0.5, opening_depth, lip_thickness)),
        origin=Origin(
            xyz=(-(opening_width * 0.5) - ((body_width - opening_width) * 0.25), 0.0, lip_thickness * 0.5)
        ),
        material=brushed_dark,
        name="intake_left_lip",
    )
    body.visual(
        Box(((body_width - opening_width) * 0.5, opening_depth, lip_thickness)),
        origin=Origin(
            xyz=((opening_width * 0.5) + ((body_width - opening_width) * 0.25), 0.0, lip_thickness * 0.5)
        ),
        material=brushed_dark,
        name="intake_right_lip",
    )
    body.visual(
        Box((0.014, opening_depth + 0.008, 0.004)),
        origin=Origin(xyz=(-(opening_width * 0.5) + 0.007, 0.0, 0.002)),
        material=brushed_dark,
        name="left_hinge_track",
    )
    body.visual(
        Box((0.014, opening_depth + 0.008, 0.004)),
        origin=Origin(xyz=((opening_width * 0.5) - 0.007, 0.0, 0.002)),
        material=brushed_dark,
        name="right_hinge_track",
    )
    body.visual(
        Box((0.300, 0.190, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.223)),
        material=brushed_dark,
        name="blower_box",
    )
    body.visual(
        Cylinder(radius=0.075, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.278)),
        material=collar_galv,
        name="duct_collar",
    )
    body.visual(
        Box((0.090, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5) - 0.001, 0.048)),
        material=brushed_dark,
        name="control_escutcheon",
    )
    body.visual(
        Box((0.003, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5) - 0.003, 0.071)),
        material=knob_marker,
        name="timer_index",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, 0.300)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
    )

    hinge_x = opening_width * 0.5
    _add_filter_cartridge(
        model,
        name="left_filter",
        body_part=body,
        hinge_origin=(-hinge_x, 0.0, 0.0),
        axis=(0.0, 1.0, 0.0),
        inward_sign=1.0,
        filter_material=brushed_dark,
        mesh_material=filter_mesh,
    )
    _add_filter_cartridge(
        model,
        name="right_filter",
        body_part=body,
        hinge_origin=(hinge_x, 0.0, 0.0),
        axis=(0.0, -1.0, 0.0),
        inward_sign=-1.0,
        filter_material=brushed_dark,
        mesh_material=filter_mesh,
    )

    knob = model.part("timer_knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_dark,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.019, length=0.022),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(0.0, -0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="grip_rim",
    )
    knob.visual(
        Box((0.004, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -0.031, 0.010,)),
        material=knob_marker,
        name="pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.042, 0.034, 0.042)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.019, 0.0)),
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, -(body_depth * 0.5), 0.048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=0.0,
            upper=5.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("insert_body")
    left_filter = object_model.get_part("left_filter")
    right_filter = object_model.get_part("right_filter")
    timer_knob = object_model.get_part("timer_knob")

    left_hinge = object_model.get_articulation("body_to_left_filter")
    right_hinge = object_model.get_articulation("body_to_right_filter")
    knob_joint = object_model.get_articulation("body_to_timer_knob")

    left_mesh = left_filter.get_visual("filter_mesh")
    right_mesh = right_filter.get_visual("filter_mesh")
    left_leaf = left_filter.get_visual("hinge_leaf")
    right_leaf = right_filter.get_visual("hinge_leaf")
    front_fascia = body.get_visual("front_fascia")
    left_track = body.get_visual("left_hinge_track")
    right_track = body.get_visual("right_hinge_track")
    shaft = timer_knob.get_visual("shaft")

    ctx.expect_gap(
        body,
        left_filter,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        negative_elem=left_mesh,
        name="left filter sits just below the intake frame",
    )
    ctx.expect_gap(
        body,
        right_filter,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        negative_elem=right_mesh,
        name="right filter sits just below the intake frame",
    )
    ctx.expect_contact(
        left_filter,
        body,
        elem_a=left_leaf,
        elem_b=left_track,
        name="left filter is supported by the left hinge track",
    )
    ctx.expect_contact(
        right_filter,
        body,
        elem_a=right_leaf,
        elem_b=right_track,
        name="right filter is supported by the right hinge track",
    )
    ctx.expect_contact(
        timer_knob,
        body,
        elem_a=shaft,
        elem_b=front_fascia,
        name="timer knob shaft meets the front fascia",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        left_closed = ctx.part_world_aabb(left_filter)
        right_closed = ctx.part_world_aabb(right_filter)

    with ctx.pose(
        {
            left_hinge: left_hinge.motion_limits.upper,
            right_hinge: right_hinge.motion_limits.upper,
        }
    ):
        left_open = ctx.part_world_aabb(left_filter)
        right_open = ctx.part_world_aabb(right_filter)

    ctx.check(
        "left filter swings downward from the intake opening",
        left_closed is not None
        and left_open is not None
        and left_open[0][2] < left_closed[0][2] - 0.12,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right filter swings downward from the intake opening",
        right_closed is not None
        and right_open is not None
        and right_open[0][2] < right_closed[0][2] - 0.12,
        details=f"closed={right_closed}, open={right_open}",
    )

    ctx.check(
        "timer knob rotates on a front-facing shaft axis",
        knob_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={knob_joint.axis}",
    )
    ctx.check(
        "timer knob has a realistic timer sweep",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower == 0.0
        and 4.5 <= knob_joint.motion_limits.upper <= 5.2,
        details=f"limits={knob_joint.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
