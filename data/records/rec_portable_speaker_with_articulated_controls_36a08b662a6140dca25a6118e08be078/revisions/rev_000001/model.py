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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_party_speaker")

    cabinet_black = model.material("cabinet_black", rgba=(0.11, 0.12, 0.13, 1.0))
    grille_black = model.material("grille_black", rgba=(0.05, 0.05, 0.06, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    control_dark = model.material("control_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    button_grey = model.material("button_grey", rgba=(0.24, 0.25, 0.27, 1.0))
    accent_ring = model.material("accent_ring", rgba=(0.33, 0.35, 0.37, 1.0))

    cabinet_width = 0.280
    cabinet_depth = 0.220
    cabinet_height = 0.540
    lower_body_height = 0.390
    top_recess_depth = 0.025
    upper_body_height = cabinet_height - lower_body_height - top_recess_depth
    control_cavity_depth = 0.028

    recess_width = 0.180
    recess_depth = 0.150
    recess_floor_z = lower_body_height + upper_body_height
    top_frame_center_z = recess_floor_z + top_recess_depth * 0.5
    side_rail_width = (cabinet_width - recess_width) * 0.5
    front_back_lip_depth = (cabinet_depth - recess_depth) * 0.5

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, lower_body_height)),
        origin=Origin(xyz=(0.0, 0.0, lower_body_height * 0.5)),
        material=cabinet_black,
        name="lower_body",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth - control_cavity_depth, upper_body_height)),
        origin=Origin(
            xyz=(
                0.0,
                -control_cavity_depth * 0.5,
                lower_body_height + upper_body_height * 0.5,
            )
        ),
        material=cabinet_black,
        name="upper_body",
    )
    cabinet.visual(
        Box((side_rail_width, cabinet_depth, top_recess_depth)),
        origin=Origin(xyz=(-(recess_width * 0.5 + side_rail_width * 0.5), 0.0, top_frame_center_z)),
        material=cabinet_black,
        name="left_top_rail",
    )
    cabinet.visual(
        Box((side_rail_width, cabinet_depth, top_recess_depth)),
        origin=Origin(xyz=((recess_width * 0.5 + side_rail_width * 0.5), 0.0, top_frame_center_z)),
        material=cabinet_black,
        name="right_top_rail",
    )
    cabinet.visual(
        Box((recess_width, front_back_lip_depth, top_recess_depth)),
        origin=Origin(xyz=(0.0, recess_depth * 0.5 + front_back_lip_depth * 0.5, top_frame_center_z)),
        material=cabinet_black,
        name="front_top_lip",
    )
    cabinet.visual(
        Box((recess_width, front_back_lip_depth, top_recess_depth)),
        origin=Origin(xyz=(0.0, -(recess_depth * 0.5 + front_back_lip_depth * 0.5), top_frame_center_z)),
        material=cabinet_black,
        name="rear_top_lip",
    )
    cabinet.visual(
        Box((recess_width, recess_depth, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, recess_floor_z + 0.0015)),
        material=trim_dark,
        name="recess_floor",
    )
    cabinet.visual(
        Box((0.224, 0.010, 0.330)),
        origin=Origin(xyz=(0.0, cabinet_depth * 0.5 - 0.0045, 0.220)),
        material=grille_black,
        name="front_grille",
    )
    cabinet.visual(
        Cylinder(radius=0.084, length=0.006),
        origin=Origin(
            xyz=(0.0, cabinet_depth * 0.5 - 0.003, 0.175),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=accent_ring,
        name="woofer_ring",
    )
    cabinet.visual(
        Cylinder(radius=0.034, length=0.005),
        origin=Origin(
            xyz=(0.0, cabinet_depth * 0.5 - 0.0025, 0.303),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=accent_ring,
        name="tweeter_ring",
    )

    cluster_width = 0.170
    cluster_height = 0.095
    frame_depth = 0.028
    frame_border = 0.012
    cluster_center_z = 0.439
    frame_center_y = cabinet_depth * 0.5 - frame_depth * 0.5
    cabinet.visual(
        Box((cluster_width, frame_depth, frame_border)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                cluster_center_z + (cluster_height - frame_border) * 0.5,
            )
        ),
        material=trim_dark,
        name="cluster_top_strip",
    )
    cabinet.visual(
        Box((cluster_width, frame_depth, frame_border)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                cluster_center_z - (cluster_height - frame_border) * 0.5,
            )
        ),
        material=trim_dark,
        name="cluster_bottom_strip",
    )
    cabinet.visual(
        Box((frame_border, frame_depth, cluster_height - 2.0 * frame_border)),
        origin=Origin(
            xyz=(
                -(cluster_width - frame_border) * 0.5,
                frame_center_y,
                cluster_center_z,
            )
        ),
        material=trim_dark,
        name="cluster_left_strip",
    )
    cabinet.visual(
        Box((frame_border, frame_depth, cluster_height - 2.0 * frame_border)),
        origin=Origin(
            xyz=(
                (cluster_width - frame_border) * 0.5,
                frame_center_y,
                cluster_center_z,
            )
        ),
        material=trim_dark,
        name="cluster_right_strip",
    )
    cabinet.visual(
        Box((0.010, frame_depth, cluster_height - 2.0 * frame_border)),
        origin=Origin(xyz=(-0.006, frame_center_y, cluster_center_z)),
        material=trim_dark,
        name="cluster_center_divider",
    )
    cabinet.visual(
        Box((0.060, frame_depth, 0.010)),
        origin=Origin(xyz=(0.044, frame_center_y, 0.441)),
        material=trim_dark,
        name="button_mid_strip",
    )
    cabinet.visual(
        Box((cluster_width - 0.020, 0.004, cluster_height - 0.018)),
        origin=Origin(xyz=(0.0, 0.084, cluster_center_z)),
        material=control_dark,
        name="control_backplate",
    )

    hinge_block_x = recess_width * 0.5 - 0.006
    hinge_block_y = -0.055
    hinge_block_z = recess_floor_z + 0.008
    cabinet.visual(
        Box((0.012, 0.020, 0.016)),
        origin=Origin(xyz=(-hinge_block_x, hinge_block_y, hinge_block_z)),
        material=trim_dark,
        name="left_handle_hinge_block",
    )
    cabinet.visual(
        Box((0.012, 0.020, 0.016)),
        origin=Origin(xyz=(hinge_block_x, hinge_block_y, hinge_block_z)),
        material=trim_dark,
        name="right_handle_hinge_block",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0055, length=0.158),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_dark,
        name="handle_shaft",
    )
    handle.visual(
        Cylinder(radius=0.0070, length=0.148),
        origin=Origin(
            xyz=(0.0, 0.070, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_dark,
        name="handle_grip",
    )
    handle.visual(
        Box((0.012, 0.070, 0.014)),
        origin=Origin(xyz=(-0.068, 0.035, 0.0)),
        material=trim_dark,
        name="left_handle_arm",
    )
    handle.visual(
        Box((0.012, 0.070, 0.014)),
        origin=Origin(xyz=(0.068, 0.035, 0.0)),
        material=trim_dark,
        name="right_handle_arm",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.158, 0.084, 0.018)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
    )

    model.articulation(
        "cabinet_to_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=handle,
        origin=Origin(xyz=(0.0, -0.055, recess_floor_z + 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.40,
        ),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(
            xyz=(0.0, 0.008, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=knob_dark,
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.017, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=control_dark,
        name="knob_face",
    )
    knob.visual(
        Box((0.003, 0.0025, 0.012)),
        origin=Origin(xyz=(0.0, 0.0205, 0.009)),
        material=button_grey,
        name="knob_indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.042, 0.022, 0.042)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
    )
    model.articulation(
        "cabinet_to_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(-0.045, 0.086, 0.441)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.20, velocity=6.0),
    )

    def add_button(name: str, center_z: float) -> None:
        button = model.part(name)
        button.visual(
            Box((0.014, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=control_dark,
            name="button_plunger",
        )
        button.visual(
            Box((0.022, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, 0.011, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.022, 0.015, 0.014)),
            mass=0.02,
            origin=Origin(xyz=(0.0, 0.0075, 0.0)),
        )
        model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(0.046, 0.086, center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    add_button("upper_button", 0.458)
    add_button("lower_button", 0.424)

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
    cabinet = object_model.get_part("cabinet")
    handle = object_model.get_part("handle")
    knob = object_model.get_part("knob")
    upper_button = object_model.get_part("upper_button")
    lower_button = object_model.get_part("lower_button")

    handle_joint = object_model.get_articulation("cabinet_to_handle")
    knob_joint = object_model.get_articulation("cabinet_to_knob")
    upper_button_joint = object_model.get_articulation("cabinet_to_upper_button")
    lower_button_joint = object_model.get_articulation("cabinet_to_lower_button")

    ctx.check(
        "handle hinge uses widthwise axis",
        tuple(round(value, 4) for value in handle_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )
    ctx.check(
        "knob turns on front-facing axis",
        tuple(round(value, 4) for value in knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={knob_joint.axis}",
    )
    ctx.check(
        "buttons plunge inward",
        tuple(round(value, 4) for value in upper_button_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(round(value, 4) for value in lower_button_joint.axis) == (0.0, -1.0, 0.0),
        details=f"upper_axis={upper_button_joint.axis}, lower_axis={lower_button_joint.axis}",
    )

    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_within(
            handle,
            cabinet,
            axes="x",
            margin=0.0,
            name="folded handle stays within cabinet width",
        )
        closed_handle_aabb = ctx.part_world_aabb(handle)
        cabinet_aabb = ctx.part_world_aabb(cabinet)

    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        opened_handle_aabb = ctx.part_world_aabb(handle)

    closed_handle_top = None if closed_handle_aabb is None else closed_handle_aabb[1][2]
    opened_handle_top = None if opened_handle_aabb is None else opened_handle_aabb[1][2]
    cabinet_top = None if cabinet_aabb is None else cabinet_aabb[1][2]
    ctx.check(
        "handle lifts clear of the top recess",
        closed_handle_top is not None
        and opened_handle_top is not None
        and cabinet_top is not None
        and closed_handle_top < cabinet_top - 0.008
        and opened_handle_top > cabinet_top + 0.040,
        details=(
            f"closed_handle_top={closed_handle_top}, "
            f"opened_handle_top={opened_handle_top}, cabinet_top={cabinet_top}"
        ),
    )

    upper_button_rest = ctx.part_world_position(upper_button)
    lower_button_rest = ctx.part_world_position(lower_button)
    with ctx.pose({upper_button_joint: upper_button_joint.motion_limits.upper}):
        upper_button_pressed = ctx.part_world_position(upper_button)
    with ctx.pose({lower_button_joint: lower_button_joint.motion_limits.upper}):
        lower_button_pressed = ctx.part_world_position(lower_button)
    ctx.check(
        "upper button moves inward when pressed",
        upper_button_rest is not None
        and upper_button_pressed is not None
        and upper_button_pressed[1] < upper_button_rest[1] - 0.003,
        details=f"rest={upper_button_rest}, pressed={upper_button_pressed}",
    )
    ctx.check(
        "lower button moves inward when pressed",
        lower_button_rest is not None
        and lower_button_pressed is not None
        and lower_button_pressed[1] < lower_button_rest[1] - 0.003,
        details=f"rest={lower_button_rest}, pressed={lower_button_pressed}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
