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


def _add_basket_visuals(
    part,
    *,
    wire_material,
    hanger_material,
    outer_x: float,
    hanger_y: float,
) -> None:
    top_z = 0.010
    bottom_z = -0.160
    side_half_x = 0.165
    side_half_y = 0.236

    part.visual(
        Box((0.330, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -side_half_y, top_z)),
        material=wire_material,
        name="basket_wire",
    )
    part.visual(
        Box((0.330, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, side_half_y, top_z)),
        material=wire_material,
    )
    part.visual(
        Box((0.008, 0.472, 0.008)),
        origin=Origin(xyz=(-side_half_x, 0.0, top_z)),
        material=wire_material,
    )
    part.visual(
        Box((0.008, 0.472, 0.008)),
        origin=Origin(xyz=(side_half_x, 0.0, top_z)),
        material=wire_material,
    )

    part.visual(
        Box((0.330, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -side_half_y, bottom_z)),
        material=wire_material,
    )
    part.visual(
        Box((0.330, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, side_half_y, bottom_z)),
        material=wire_material,
    )
    part.visual(
        Box((0.008, 0.472, 0.008)),
        origin=Origin(xyz=(-side_half_x, 0.0, bottom_z)),
        material=wire_material,
    )
    part.visual(
        Box((0.008, 0.472, 0.008)),
        origin=Origin(xyz=(side_half_x, 0.0, bottom_z)),
        material=wire_material,
    )

    for x, y in (
        (-side_half_x, -side_half_y),
        (side_half_x, -side_half_y),
        (-side_half_x, side_half_y),
        (side_half_x, side_half_y),
        (0.0, -side_half_y),
        (0.0, side_half_y),
    ):
        part.visual(
            Box((0.008, 0.008, 0.172)),
            origin=Origin(xyz=(x, y, -0.075)),
            material=wire_material,
        )

    for y in (-0.106, 0.0, 0.106):
        part.visual(
            Box((0.334, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, y, -0.154)),
            material=wire_material,
        )
    for x in (-0.110, 0.0, 0.110):
        part.visual(
            Box((0.006, 0.476, 0.006)),
            origin=Origin(xyz=(x, 0.0, -0.154)),
            material=wire_material,
        )

    part.visual(
        Box((0.060, 0.224, 0.012)),
        origin=Origin(xyz=(outer_x, hanger_y, 0.006)),
        material=hanger_material,
        name="outer_hanger",
    )
    part.visual(
        Box((0.060, 0.224, 0.012)),
        origin=Origin(xyz=(-outer_x, hanger_y, 0.006)),
        material=hanger_material,
        name="inner_hanger",
    )
    for x in (outer_x, -outer_x):
        side_x = -side_half_x if x < 0.0 else side_half_x
        part.visual(
            Box((abs(x - side_x) + 0.014, 0.020, 0.012)),
            origin=Origin(xyz=((x + side_x) * 0.5, hanger_y, 0.008)),
            material=hanger_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deli_display_freezer")

    body_white = model.material("body_white", rgba=(0.92, 0.94, 0.96, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.28, 0.30, 0.33, 1.0))
    liner_white = model.material("liner_white", rgba=(0.84, 0.87, 0.90, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.16, 0.17, 0.18, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.70, 0.84, 0.92, 0.30))
    basket_steel = model.material("basket_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    dial_black = model.material("dial_black", rgba=(0.14, 0.14, 0.15, 1.0))
    pointer_red = model.material("pointer_red", rgba=(0.82, 0.24, 0.16, 1.0))
    foot_black = model.material("foot_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.980, 0.620, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=trim_grey,
        name="plinth",
    )
    for index, (fx, fy) in enumerate(
        (
            (-0.410, -0.250),
            (0.410, -0.250),
            (-0.410, 0.250),
            (0.410, 0.250),
        )
    ):
        body.visual(
            Cylinder(radius=0.026, length=0.040),
            origin=Origin(xyz=(fx, fy, 0.020)),
            material=foot_black,
            name=f"foot_{index}",
        )

    body.visual(
        Box((1.020, 0.680, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=body_white,
        name="base_shell",
    )
    body.visual(
        Box((1.020, 0.060, 0.680)),
        origin=Origin(xyz=(0.0, -0.340, 0.440)),
        material=body_white,
        name="front_shell",
    )
    body.visual(
        Box((1.020, 0.050, 0.680)),
        origin=Origin(xyz=(0.0, 0.345, 0.440)),
        material=body_white,
        name="rear_shell",
    )
    body.visual(
        Box((0.065, 0.630, 0.680)),
        origin=Origin(xyz=(-0.4775, 0.0, 0.440)),
        material=body_white,
        name="left_shell",
    )
    body.visual(
        Box((0.065, 0.630, 0.680)),
        origin=Origin(xyz=(0.4775, 0.0, 0.440)),
        material=body_white,
        name="right_shell",
    )

    body.visual(
        Box((0.880, 0.540, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.2125)),
        material=liner_white,
        name="liner_floor",
    )
    body.visual(
        Box((0.015, 0.540, 0.520)),
        origin=Origin(xyz=(-0.4475, 0.0, 0.480)),
        material=liner_white,
        name="left_liner",
    )
    body.visual(
        Box((0.015, 0.540, 0.520)),
        origin=Origin(xyz=(0.4475, 0.0, 0.480)),
        material=liner_white,
        name="right_liner",
    )
    body.visual(
        Box((0.880, 0.015, 0.520)),
        origin=Origin(xyz=(0.0, -0.2775, 0.480)),
        material=liner_white,
        name="front_liner",
    )
    body.visual(
        Box((0.880, 0.015, 0.520)),
        origin=Origin(xyz=(0.0, 0.2775, 0.480)),
        material=liner_white,
        name="rear_liner",
    )

    body.visual(
        Box((0.085, 0.780, 0.022)),
        origin=Origin(xyz=(-0.4675, 0.060, 0.795)),
        material=trim_grey,
        name="left_guide_track",
    )
    body.visual(
        Box((0.085, 0.780, 0.022)),
        origin=Origin(xyz=(0.4675, 0.060, 0.795)),
        material=trim_grey,
        name="right_guide_track",
    )
    body.visual(
        Box((0.884, 0.050, 0.026)),
        origin=Origin(xyz=(0.0, -0.310, 0.793)),
        material=trim_grey,
        name="front_rim",
    )
    body.visual(
        Box((0.940, 0.050, 0.036)),
        origin=Origin(xyz=(0.0, 0.405, 0.794)),
        material=trim_grey,
        name="rear_bridge",
    )

    body.visual(
        Box((0.030, 0.560, 0.014)),
        origin=Origin(xyz=(-0.425, 0.0, 0.666)),
        material=trim_grey,
        name="left_basket_support",
    )
    body.visual(
        Box((0.028, 0.560, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.666)),
        material=trim_grey,
        name="center_basket_support",
    )
    body.visual(
        Box((0.030, 0.560, 0.014)),
        origin=Origin(xyz=(0.425, 0.0, 0.666)),
        material=trim_grey,
        name="right_basket_support",
    )

    for index, z in enumerate((0.300, 0.340, 0.380, 0.420)):
        body.visual(
            Box((0.190, 0.008, 0.016)),
            origin=Origin(xyz=(-0.120, -0.370, z)),
            material=dark_grey,
            name=f"vent_slat_{index}",
        )

    body.visual(
        Cylinder(radius=0.036, length=0.016),
        origin=Origin(
            xyz=(0.340, -0.378, 0.460),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_grey,
        name="thermostat_boss",
    )
    body.visual(
        Box((0.090, 0.006, 0.040)),
        origin=Origin(xyz=(0.338, -0.367, 0.458)),
        material=dark_grey,
        name="thermostat_scale",
    )

    body.inertial = Inertial.from_geometry(
        Box((1.100, 0.740, 0.840)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
    )

    lid = model.part("sliding_lid")
    lid.visual(
        Box((0.022, 0.600, 0.008)),
        origin=Origin(xyz=(-0.447, 0.0, 0.004)),
        material=dark_grey,
        name="left_runner",
    )
    lid.visual(
        Box((0.022, 0.600, 0.008)),
        origin=Origin(xyz=(0.447, 0.0, 0.004)),
        material=dark_grey,
        name="right_runner",
    )
    lid.visual(
        Box((0.032, 0.620, 0.026)),
        origin=Origin(xyz=(-0.446, 0.0, 0.021)),
        material=trim_grey,
        name="left_frame",
    )
    lid.visual(
        Box((0.032, 0.620, 0.026)),
        origin=Origin(xyz=(0.446, 0.0, 0.021)),
        material=trim_grey,
        name="right_frame",
    )
    lid.visual(
        Box((0.892, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, -0.295, 0.021)),
        material=trim_grey,
        name="front_frame",
    )
    lid.visual(
        Box((0.892, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.295, 0.021)),
        material=trim_grey,
        name="rear_frame",
    )
    lid.visual(
        Box((0.848, 0.560, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=glass_tint,
        name="glass_panel",
    )
    lid.visual(
        Box((0.220, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -0.310, 0.033)),
        material=dark_grey,
        name="pull_handle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.900, 0.620, 0.045)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    dial = model.part("thermostat_dial")
    dial.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(
            xyz=(0.0, -0.010, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_grey,
        name="dial_shaft",
    )
    dial.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(
            xyz=(0.0, -0.033, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dial_black,
        name="dial_body",
    )
    dial.visual(
        Box((0.016, 0.004, 0.006)),
        origin=Origin(xyz=(0.015, -0.046, 0.0)),
        material=pointer_red,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.080, 0.060, 0.080)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
    )

    left_basket = model.part("left_basket")
    _add_basket_visuals(
        left_basket,
        wire_material=basket_steel,
        hanger_material=trim_grey,
        outer_x=-0.210,
        hanger_y=-0.124,
    )
    left_basket.inertial = Inertial.from_geometry(
        Box((0.420, 0.500, 0.190)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    right_basket = model.part("right_basket")
    _add_basket_visuals(
        right_basket,
        wire_material=basket_steel,
        hanger_material=trim_grey,
        outer_x=0.210,
        hanger_y=0.124,
    )
    right_basket.inertial = Inertial.from_geometry(
        Box((0.420, 0.500, 0.190)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    lid_slide = model.articulation(
        "body_to_sliding_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.806)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=0.240,
        ),
    )
    dial_joint = model.articulation(
        "body_to_thermostat_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.340, -0.386, 0.460)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "body_to_left_basket",
        ArticulationType.FIXED,
        parent=body,
        child=left_basket,
        origin=Origin(xyz=(-0.205, 0.0, 0.673)),
    )
    model.articulation(
        "body_to_right_basket",
        ArticulationType.FIXED,
        parent=body,
        child=right_basket,
        origin=Origin(xyz=(0.205, 0.0, 0.673)),
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

    body = object_model.get_part("body")
    lid = object_model.get_part("sliding_lid")
    dial = object_model.get_part("thermostat_dial")
    left_basket = object_model.get_part("left_basket")
    right_basket = object_model.get_part("right_basket")
    lid_slide = object_model.get_articulation("body_to_sliding_lid")
    dial_joint = object_model.get_articulation("body_to_thermostat_dial")

    ctx.expect_contact(
        lid,
        body,
        elem_a="left_runner",
        elem_b="left_guide_track",
        name="left lid runner sits on left guide",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="right_runner",
        elem_b="right_guide_track",
        name="right lid runner sits on right guide",
    )

    lid_rest = ctx.part_world_position(lid)
    with ctx.pose({lid_slide: lid_slide.motion_limits.upper}):
        ctx.expect_overlap(
            lid,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="left_guide_track",
            min_overlap=0.45,
            name="lid keeps substantial guide engagement when open",
        )
        lid_open = ctx.part_world_position(lid)

    ctx.check(
        "lid slides toward the rear",
        lid_rest is not None
        and lid_open is not None
        and lid_open[1] > lid_rest[1] + 0.18,
        details=f"rest={lid_rest}, open={lid_open}",
    )

    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_shaft",
        elem_b="thermostat_boss",
        name="thermostat dial mounts on the front boss",
    )
    ctx.check(
        "thermostat dial rotates about the front-face axis",
        dial_joint.axis == (0.0, -1.0, 0.0)
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is not None
        and dial_joint.motion_limits.upper is not None
        and dial_joint.motion_limits.lower < 0.0 < dial_joint.motion_limits.upper,
        details=(
            f"axis={dial_joint.axis}, "
            f"limits={dial_joint.motion_limits}"
        ),
    )

    ctx.expect_contact(
        left_basket,
        body,
        elem_a="outer_hanger",
        elem_b="left_basket_support",
        name="left basket rests on the outer support rail",
    )
    ctx.expect_contact(
        left_basket,
        body,
        elem_a="inner_hanger",
        elem_b="center_basket_support",
        name="left basket rests on the center support rail",
    )
    ctx.expect_contact(
        right_basket,
        body,
        elem_a="inner_hanger",
        elem_b="center_basket_support",
        name="right basket rests on the center support rail",
    )
    ctx.expect_contact(
        right_basket,
        body,
        elem_a="outer_hanger",
        elem_b="right_basket_support",
        name="right basket rests on the outer support rail",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
