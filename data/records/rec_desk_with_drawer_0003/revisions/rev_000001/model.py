from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except FileNotFoundError:
            pass
        return "/"


os.getcwd = _safe_getcwd

_REAL_PATH_CWD = pathlib.Path.cwd
_REAL_PATH_ABSOLUTE = pathlib.Path.absolute
_REAL_PATH_RESOLVE = pathlib.Path.resolve


def _safe_path_cwd(cls):
    try:
        return _REAL_PATH_CWD.__func__(cls)
    except FileNotFoundError:
        return cls("/")


def _safe_path_absolute(self):
    try:
        return _REAL_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        return self if self.is_absolute() else type(self)("/") / self


def _safe_path_resolve(self, strict: bool = False):
    try:
        return _REAL_PATH_RESOLVE(self, strict=strict)
    except FileNotFoundError:
        candidate = self if self.is_absolute() else type(self)("/") / self
        return candidate


pathlib.Path.cwd = classmethod(_safe_path_cwd)
pathlib.Path.absolute = _safe_path_absolute
pathlib.Path.resolve = _safe_path_resolve
if hasattr(pathlib, "PosixPath"):
    pathlib.PosixPath.cwd = classmethod(_safe_path_cwd)
    pathlib.PosixPath.absolute = _safe_path_absolute
    pathlib.PosixPath.resolve = _safe_path_resolve

__file__ = "/model.py"

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


TOP_WIDTH = 1.20
TOP_DEPTH = 0.60
TOP_THICKNESS = 0.04
TOP_CENTER_Z = 0.73
TOP_UNDERSIDE_Z = TOP_CENTER_Z - TOP_THICKNESS / 2.0

LEG_HEIGHT = TOP_UNDERSIDE_Z
LEG_TOP = 0.05
LEG_BOTTOM = 0.03
LEG_X = 0.515
LEG_Y = 0.235

APRON_HEIGHT = 0.09
APRON_FRONT_BACK_THICKNESS = 0.025
APRON_SIDE_THICKNESS = 0.025
APRON_SIDE_LENGTH = LEG_Y * 2.0 - LEG_TOP
APRON_CENTER_Z = TOP_UNDERSIDE_Z - APRON_HEIGHT / 2.0
APRON_FRONT_Y = -0.2475
APRON_BACK_Y = 0.2475
APRON_SIDE_X = LEG_X - LEG_TOP / 2.0 - APRON_SIDE_THICKNESS / 2.0

DRAWER_FACE_WIDTH = 0.44
DRAWER_FACE_HEIGHT = 0.09
DRAWER_FACE_THICKNESS = 0.02
DRAWER_BOX_WIDTH = 0.41
DRAWER_BOX_HEIGHT = 0.055
DRAWER_BOX_LENGTH = 0.37
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_FRONT_PANEL_THICKNESS = 0.012
DRAWER_BACK_PANEL_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.008
DRAWER_WALL_HEIGHT = DRAWER_BOX_HEIGHT - DRAWER_BOTTOM_THICKNESS
DRAWER_BOX_FRONT_Y = DRAWER_FACE_THICKNESS / 2.0
DRAWER_BOX_REAR_Y = DRAWER_BOX_FRONT_Y + DRAWER_BOX_LENGTH
DRAWER_SIDE_LENGTH = (
    DRAWER_BOX_LENGTH - DRAWER_FRONT_PANEL_THICKNESS - DRAWER_BACK_PANEL_THICKNESS
)
DRAWER_SIDE_CENTER_Y = (
    DRAWER_BOX_FRONT_Y
    + DRAWER_FRONT_PANEL_THICKNESS
    + DRAWER_SIDE_LENGTH / 2.0
)
DRAWER_FRONT_PANEL_CENTER_Y = (
    DRAWER_BOX_FRONT_Y + DRAWER_FRONT_PANEL_THICKNESS / 2.0
)
DRAWER_BACK_PANEL_CENTER_Y = DRAWER_BOX_REAR_Y - DRAWER_BACK_PANEL_THICKNESS / 2.0
DRAWER_BOX_CENTER_Z = -0.0025
DRAWER_BOX_BOTTOM_Z = DRAWER_BOX_CENTER_Z - DRAWER_BOX_HEIGHT / 2.0
DRAWER_WALL_CENTER_Z = DRAWER_BOX_BOTTOM_Z + DRAWER_BOTTOM_THICKNESS + DRAWER_WALL_HEIGHT / 2.0
DRAWER_BOTTOM_CENTER_Z = DRAWER_BOX_BOTTOM_Z + DRAWER_BOTTOM_THICKNESS / 2.0
DRAWER_ORIGIN_Y = APRON_FRONT_Y
DRAWER_ORIGIN_Z = 0.655

GUIDE_RAIL_WIDTH = 0.014
GUIDE_RAIL_HEIGHT = 0.055
GUIDE_RAIL_LENGTH = 0.34
GUIDE_RAIL_CENTER_X = 0.222
GUIDE_RAIL_CENTER_Y = -0.02
GUIDE_RAIL_CENTER_Z = TOP_UNDERSIDE_Z - GUIDE_RAIL_HEIGHT / 2.0

RUNNER_WIDTH = 0.010
RUNNER_HEIGHT = 0.03
RUNNER_LENGTH = 0.30
RUNNER_CENTER_X = 0.210
RUNNER_CENTER_Y = 0.18
RUNNER_CENTER_Z = 0.010

LEG_LOWER_TAPER = 0.036
LEG_UPPER_TAPER = 0.043
LEG_SEGMENT_1_HEIGHT = 0.22
LEG_SEGMENT_2_HEIGHT = 0.20
LEG_SEGMENT_3_HEIGHT = 0.16
LEG_SEGMENT_4_HEIGHT = LEG_HEIGHT - (
    LEG_SEGMENT_1_HEIGHT + LEG_SEGMENT_2_HEIGHT + LEG_SEGMENT_3_HEIGHT
)
LEG_SEGMENT_2_SIZE = 0.035
LEG_SEGMENT_3_SIZE = 0.042

DRAWER_PULL_RADIUS = 0.010
DRAWER_PULL_LENGTH = 0.022
DRAWER_PULL_CENTER_Y = -(
    DRAWER_FACE_THICKNESS / 2.0 + DRAWER_PULL_LENGTH / 2.0 - 0.002
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="writing_desk")

    walnut = model.material("walnut", rgba=(0.43, 0.28, 0.18, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.30, 0.20, 0.13, 1.0))
    drawer_interior = model.material("drawer_interior", rgba=(0.57, 0.44, 0.31, 1.0))
    drawer_slide = model.material("drawer_slide", rgba=(0.36, 0.37, 0.39, 1.0))
    antique_brass = model.material("antique_brass", rgba=(0.67, 0.56, 0.34, 1.0))

    desk_frame = model.part("desk_frame")
    desk_frame.visual(
        Box((TOP_WIDTH, TOP_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_CENTER_Z)),
        material=walnut,
        name="top_panel",
    )
    desk_frame.visual(
        Box((0.26, APRON_FRONT_BACK_THICKNESS, APRON_HEIGHT)),
        origin=Origin(xyz=(-0.36, APRON_FRONT_Y, APRON_CENTER_Z)),
        material=walnut_dark,
        name="left_front_apron",
    )
    desk_frame.visual(
        Box((0.26, APRON_FRONT_BACK_THICKNESS, APRON_HEIGHT)),
        origin=Origin(xyz=(0.36, APRON_FRONT_Y, APRON_CENTER_Z)),
        material=walnut_dark,
        name="right_front_apron",
    )
    desk_frame.visual(
        Box((0.955, APRON_FRONT_BACK_THICKNESS, APRON_HEIGHT)),
        origin=Origin(xyz=(0.0, APRON_BACK_Y, APRON_CENTER_Z)),
        material=walnut_dark,
        name="back_apron",
    )
    desk_frame.visual(
        Box((APRON_SIDE_THICKNESS, APRON_SIDE_LENGTH, APRON_HEIGHT)),
        origin=Origin(xyz=(-APRON_SIDE_X, 0.0, APRON_CENTER_Z)),
        material=walnut_dark,
        name="left_side_apron",
    )
    desk_frame.visual(
        Box((APRON_SIDE_THICKNESS, APRON_SIDE_LENGTH, APRON_HEIGHT)),
        origin=Origin(xyz=(APRON_SIDE_X, 0.0, APRON_CENTER_Z)),
        material=walnut_dark,
        name="right_side_apron",
    )
    desk_frame.visual(
        Box((GUIDE_RAIL_WIDTH, GUIDE_RAIL_LENGTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_RAIL_CENTER_X, GUIDE_RAIL_CENTER_Y, GUIDE_RAIL_CENTER_Z)),
        material=drawer_slide,
        name="left_guide_rail",
    )
    desk_frame.visual(
        Box((GUIDE_RAIL_WIDTH, GUIDE_RAIL_LENGTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(GUIDE_RAIL_CENTER_X, GUIDE_RAIL_CENTER_Y, GUIDE_RAIL_CENTER_Z)),
        material=drawer_slide,
        name="right_guide_rail",
    )
    desk_frame.inertial = Inertial.from_geometry(
        Box((TOP_WIDTH, TOP_DEPTH, 0.13)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
    )

    leg_positions = {
        "front_left_leg": (-LEG_X, -LEG_Y, 0.0),
        "front_right_leg": (LEG_X, -LEG_Y, 0.0),
        "rear_left_leg": (-LEG_X, LEG_Y, 0.0),
        "rear_right_leg": (LEG_X, LEG_Y, 0.0),
    }
    for leg_name, leg_xyz in leg_positions.items():
        leg = model.part(leg_name)
        leg.visual(
            Box((LEG_BOTTOM, LEG_BOTTOM, LEG_SEGMENT_1_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, LEG_SEGMENT_1_HEIGHT / 2.0)),
            material=walnut,
            name="leg_lower",
        )
        leg.visual(
            Box((LEG_SEGMENT_2_SIZE, LEG_SEGMENT_2_SIZE, LEG_SEGMENT_2_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    LEG_SEGMENT_1_HEIGHT + LEG_SEGMENT_2_HEIGHT / 2.0,
                )
            ),
            material=walnut,
            name="leg_mid_lower",
        )
        leg.visual(
            Box((LEG_SEGMENT_3_SIZE, LEG_SEGMENT_3_SIZE, LEG_SEGMENT_3_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    LEG_SEGMENT_1_HEIGHT
                    + LEG_SEGMENT_2_HEIGHT
                    + LEG_SEGMENT_3_HEIGHT / 2.0,
                )
            ),
            material=walnut,
            name="leg_mid_upper",
        )
        leg.visual(
            Box((LEG_TOP, LEG_TOP, LEG_SEGMENT_4_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    LEG_SEGMENT_1_HEIGHT
                    + LEG_SEGMENT_2_HEIGHT
                    + LEG_SEGMENT_3_HEIGHT
                    + LEG_SEGMENT_4_HEIGHT / 2.0,
                )
            ),
            material=walnut,
            name="leg_upper",
        )
        leg.inertial = Inertial.from_geometry(
            Box((LEG_TOP, LEG_TOP, LEG_HEIGHT)),
            mass=2.0,
            origin=Origin(xyz=(0.0, 0.0, LEG_HEIGHT / 2.0)),
        )
        model.articulation(
            f"desk_frame_to_{leg_name}",
            ArticulationType.FIXED,
            parent=desk_frame,
            child=leg,
            origin=Origin(xyz=leg_xyz),
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS, DRAWER_FACE_HEIGHT)),
        material=walnut_dark,
        name="drawer_face",
    )
    drawer.visual(
        Cylinder(radius=DRAWER_PULL_RADIUS, length=DRAWER_PULL_LENGTH),
        origin=Origin(xyz=(0.0, DRAWER_PULL_CENTER_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=antique_brass,
        name="drawer_pull",
    )
    drawer.visual(
        Box((DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS, DRAWER_FRONT_PANEL_THICKNESS, DRAWER_WALL_HEIGHT)),
        origin=Origin(xyz=(0.0, DRAWER_FRONT_PANEL_CENTER_Y, DRAWER_WALL_CENTER_Z)),
        material=drawer_interior,
        name="drawer_front_panel",
    )
    drawer.visual(
        Box((DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS, DRAWER_BACK_PANEL_THICKNESS, DRAWER_WALL_HEIGHT)),
        origin=Origin(xyz=(0.0, DRAWER_BACK_PANEL_CENTER_Y, DRAWER_WALL_CENTER_Z)),
        material=drawer_interior,
        name="drawer_back_panel",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_SIDE_LENGTH, DRAWER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(DRAWER_BOX_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0),
                DRAWER_SIDE_CENTER_Y,
                DRAWER_WALL_CENTER_Z,
            )
        ),
        material=drawer_interior,
        name="left_drawer_side",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_SIDE_LENGTH, DRAWER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                DRAWER_BOX_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0,
                DRAWER_SIDE_CENTER_Y,
                DRAWER_WALL_CENTER_Z,
            )
        ),
        material=drawer_interior,
        name="right_drawer_side",
    )
    drawer.visual(
        Box(
            (
                DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                DRAWER_SIDE_LENGTH,
                DRAWER_BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(xyz=(0.0, DRAWER_SIDE_CENTER_Y, DRAWER_BOTTOM_CENTER_Z)),
        material=drawer_interior,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(-RUNNER_CENTER_X, RUNNER_CENTER_Y, RUNNER_CENTER_Z)),
        material=drawer_slide,
        name="left_runner",
    )
    drawer.visual(
        Box((RUNNER_WIDTH, RUNNER_LENGTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(RUNNER_CENTER_X, RUNNER_CENTER_Y, RUNNER_CENTER_Z)),
        material=drawer_slide,
        name="right_runner",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FACE_WIDTH, 0.39, DRAWER_FACE_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.185, 0.0)),
    )

    model.articulation(
        "desk_to_drawer",
        ArticulationType.PRISMATIC,
        parent=desk_frame,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_ORIGIN_Y, DRAWER_ORIGIN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.35,
            lower=0.0,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk_frame = object_model.get_part("desk_frame")
    front_left_leg = object_model.get_part("front_left_leg")
    front_right_leg = object_model.get_part("front_right_leg")
    rear_left_leg = object_model.get_part("rear_left_leg")
    rear_right_leg = object_model.get_part("rear_right_leg")
    drawer = object_model.get_part("drawer")
    drawer_slide_joint = object_model.get_articulation("desk_to_drawer")

    top_panel = desk_frame.get_visual("top_panel")
    left_guide_rail = desk_frame.get_visual("left_guide_rail")
    right_guide_rail = desk_frame.get_visual("right_guide_rail")
    drawer_face = drawer.get_visual("drawer_face")
    left_runner = drawer.get_visual("left_runner")
    right_runner = drawer.get_visual("right_runner")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for leg_name, leg in (
        ("front_left_leg", front_left_leg),
        ("front_right_leg", front_right_leg),
        ("rear_left_leg", rear_left_leg),
        ("rear_right_leg", rear_right_leg),
    ):
        ctx.expect_contact(leg, desk_frame, name=f"{leg_name}_mounted_to_frame")

    ctx.expect_origin_distance(
        front_left_leg,
        front_right_leg,
        axes="x",
        min_dist=1.02,
        max_dist=1.04,
        name="front_leg_span",
    )
    ctx.expect_origin_distance(
        rear_left_leg,
        rear_right_leg,
        axes="x",
        min_dist=1.02,
        max_dist=1.04,
        name="rear_leg_span",
    )
    ctx.expect_origin_distance(
        front_left_leg,
        rear_left_leg,
        axes="y",
        min_dist=0.46,
        max_dist=0.48,
        name="left_leg_depth",
    )
    ctx.expect_origin_distance(
        front_right_leg,
        rear_right_leg,
        axes="y",
        min_dist=0.46,
        max_dist=0.48,
        name="right_leg_depth",
    )

    with ctx.pose({drawer_slide_joint: 0.0}):
        ctx.expect_contact(
            drawer,
            desk_frame,
            elem_a=left_runner,
            elem_b=left_guide_rail,
            name="left_runner_contacts_left_rail_closed",
        )
        ctx.expect_contact(
            drawer,
            desk_frame,
            elem_a=right_runner,
            elem_b=right_guide_rail,
            name="right_runner_contacts_right_rail_closed",
        )
        ctx.expect_gap(
            desk_frame,
            drawer,
            axis="z",
            min_gap=0.008,
            max_gap=0.015,
            positive_elem=top_panel,
            name="drawer_clears_top_closed",
        )
        ctx.expect_overlap(
            drawer,
            desk_frame,
            axes="x",
            min_overlap=0.40,
            elem_a=drawer_face,
            elem_b=top_panel,
            name="drawer_centered_under_top_closed",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="drawer_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="drawer_closed_no_floating")
        closed_position = ctx.part_world_position(drawer)

    limits = drawer_slide_joint.motion_limits
    assert limits is not None and limits.upper is not None

    with ctx.pose({drawer_slide_joint: limits.upper}):
        ctx.expect_contact(
            drawer,
            desk_frame,
            elem_a=left_runner,
            elem_b=left_guide_rail,
            name="left_runner_contacts_left_rail_open",
        )
        ctx.expect_contact(
            drawer,
            desk_frame,
            elem_a=right_runner,
            elem_b=right_guide_rail,
            name="right_runner_contacts_right_rail_open",
        )
        ctx.expect_gap(
            desk_frame,
            drawer,
            axis="z",
            min_gap=0.008,
            max_gap=0.015,
            positive_elem=top_panel,
            name="drawer_clears_top_open",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="drawer_open_no_overlap")
        ctx.fail_if_isolated_parts(name="drawer_open_no_floating")
        open_position = ctx.part_world_position(drawer)

    assert closed_position is not None
    assert open_position is not None
    delta_x = open_position[0] - closed_position[0]
    delta_y = open_position[1] - closed_position[1]
    delta_z = open_position[2] - closed_position[2]
    ctx.check(
        "drawer_prismatic_axis",
        abs(delta_x) <= 1e-6
        and abs(delta_z) <= 1e-6
        and abs(delta_y + limits.upper) <= 1e-6,
        (
            "Drawer should slide straight forward along -Y by its joint travel; "
            f"observed delta=({delta_x:.6f}, {delta_y:.6f}, {delta_z:.6f})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
