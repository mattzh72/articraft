from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

# The harness sometimes evaluates this script from a deleted temporary working
# directory. pathlib.Path.resolve() then fails deep inside os.getcwd(). Install a
# process-local fallback before importing sdk or any helper that may resolve
# relative paths.
_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except OSError:
            pass
        return "/"


os.getcwd = _safe_getcwd
os.chdir("/")

_REAL_PATH_ABSOLUTE = pathlib.Path.absolute
_REAL_PATH_CWD = pathlib.Path.cwd


def _safe_path_absolute(self):
    try:
        return _REAL_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        parts = list(self.parts)
        if self.is_absolute():
            return self
        return self.__class__("/", *parts)


@classmethod
def _safe_path_cwd(cls):
    return cls("/")


pathlib.Path.absolute = _safe_path_absolute
pathlib.Path.cwd = _safe_path_cwd

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

SCRIPT_FILE = os.path.abspath(globals().get("__file__", "model.py"))
__file__ = SCRIPT_FILE
HERE = os.path.dirname(SCRIPT_FILE) or "/"
ASSETS = AssetContext.from_script(SCRIPT_FILE)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_electric_oven", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.08, 0.10, 0.12, 0.78))
    gloss_glass = model.material("gloss_glass", rgba=(0.22, 0.26, 0.30, 0.38))
    cavity_enamel = model.material("cavity_enamel", rgba=(0.10, 0.10, 0.11, 1.0))

    bezel_w = 0.64
    bezel_h = 0.66
    carcass_w = 0.58
    carcass_h = 0.60
    body_depth = 0.48
    shell_t = 0.024
    back_t = 0.018
    bezel_t = 0.03
    body_bottom_z = 0.03
    carcass_center_z = body_bottom_z + (carcass_h / 2.0)
    front_frame_y = (body_depth / 2.0) + (bezel_t / 2.0)

    bottom_sill_h = 0.055
    top_fascia_h = 0.095
    opening_w = 0.56
    opening_h = bezel_h - bottom_sill_h - top_fascia_h
    door_w = opening_w - 0.004
    door_h = opening_h - 0.004
    door_t = 0.042
    door_bottom_z = bottom_sill_h + 0.002
    hinge_y = front_frame_y + (bezel_t / 2.0) + 0.0025
    hinge_z = door_bottom_z
    hinge_offset_x = (door_w / 2.0) - 0.030

    side_frame_w = (bezel_w - opening_w) / 2.0

    body = model.part("housing")
    body.visual(
        Box((shell_t, body_depth, carcass_h)),
        origin=Origin(xyz=(-(carcass_w / 2.0) + (shell_t / 2.0), 0.0, carcass_center_z)),
        material=stainless,
        name="left_shell",
    )
    body.visual(
        Box((shell_t, body_depth, carcass_h)),
        origin=Origin(xyz=((carcass_w / 2.0) - (shell_t / 2.0), 0.0, carcass_center_z)),
        material=stainless,
        name="right_shell",
    )
    body.visual(
        Box((carcass_w - (2.0 * shell_t), body_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom_z + (shell_t / 2.0))),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((carcass_w - (2.0 * shell_t), body_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom_z + carcass_h - (shell_t / 2.0))),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((carcass_w - (2.0 * shell_t), back_t, carcass_h - (2.0 * shell_t))),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth / 2.0) + (back_t / 2.0),
                body_bottom_z + shell_t + ((carcass_h - (2.0 * shell_t)) / 2.0),
            )
        ),
        material=stainless,
        name="back_shell",
    )
    body.visual(
        Box((side_frame_w, bezel_t, bezel_h)),
        origin=Origin(xyz=(-(bezel_w / 2.0) + (side_frame_w / 2.0), front_frame_y, bezel_h / 2.0)),
        material=stainless,
        name="front_left_frame",
    )
    body.visual(
        Box((side_frame_w, bezel_t, bezel_h)),
        origin=Origin(xyz=((bezel_w / 2.0) - (side_frame_w / 2.0), front_frame_y, bezel_h / 2.0)),
        material=stainless,
        name="front_right_frame",
    )
    body.visual(
        Box((opening_w, bezel_t, bottom_sill_h)),
        origin=Origin(xyz=(0.0, front_frame_y, bottom_sill_h / 2.0)),
        material=stainless,
        name="bottom_sill",
    )
    body.visual(
        Box((opening_w, bezel_t, top_fascia_h)),
        origin=Origin(xyz=(0.0, front_frame_y, bezel_h - (top_fascia_h / 2.0))),
        material=stainless,
        name="top_fascia",
    )
    body.visual(
        Box((0.22, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, front_frame_y + (bezel_t / 2.0) + 0.004, bezel_h - 0.048)),
        material=gloss_glass,
        name="display_window",
    )

    liner_t = 0.004
    liner_depth = body_depth - back_t - 0.02
    liner_center_y = -(0.01 - (back_t / 2.0))
    liner_clear_w = carcass_w - (2.0 * shell_t)
    liner_inner_w = liner_clear_w - (2.0 * liner_t)
    liner_h = opening_h - 0.042
    liner_center_z = bottom_sill_h + (opening_h / 2.0)
    body.visual(
        Box((liner_t, liner_depth, liner_h)),
        origin=Origin(
            xyz=(
                -(carcass_w / 2.0) + shell_t + (liner_t / 2.0),
                liner_center_y,
                liner_center_z,
            )
        ),
        material=cavity_enamel,
        name="cavity_left",
    )
    body.visual(
        Box((liner_t, liner_depth, liner_h)),
        origin=Origin(
            xyz=(
                (carcass_w / 2.0) - shell_t - (liner_t / 2.0),
                liner_center_y,
                liner_center_z,
            )
        ),
        material=cavity_enamel,
        name="cavity_right",
    )
    body.visual(
        Box((liner_inner_w, liner_depth, liner_t)),
        origin=Origin(xyz=(0.0, liner_center_y, bottom_sill_h + 0.021 + (liner_t / 2.0))),
        material=cavity_enamel,
        name="cavity_floor",
    )
    body.visual(
        Box((liner_inner_w, liner_depth, liner_t)),
        origin=Origin(
            xyz=(
                0.0,
                liner_center_y,
                bottom_sill_h + opening_h - 0.021 - (liner_t / 2.0),
            )
        ),
        material=cavity_enamel,
        name="cavity_ceiling",
    )
    body.visual(
        Box((liner_inner_w, liner_t, liner_h)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth / 2.0) + back_t + (liner_t / 2.0),
                liner_center_z,
            )
        ),
        material=cavity_enamel,
        name="cavity_back",
    )
    hinge_pin_length = 0.020
    hinge_pin_radius = 0.004
    for side, sign in (("left", -1.0), ("right", 1.0)):
        body.visual(
            Cylinder(radius=hinge_pin_radius, length=hinge_pin_length),
            origin=Origin(
                xyz=(sign * hinge_offset_x, hinge_y, hinge_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=stainless,
            name=f"{side}_hinge_pin",
        )
    body.inertial = Inertial.from_geometry(
        Box((bezel_w, body_depth + bezel_t, bezel_h)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, bezel_h / 2.0)),
    )

    door = model.part("door")
    glass_w = door_w - 0.060
    glass_h = door_h - 0.110
    glass_center_z = (door_h / 2.0) + 0.005
    hinge_knuckle_length = 0.024
    hinge_knuckle_radius = 0.0055
    for side, sign in (("left", -1.0), ("right", 1.0)):
        door.visual(
            Cylinder(radius=hinge_knuckle_radius, length=hinge_knuckle_length),
            origin=Origin(
                xyz=(sign * hinge_offset_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=stainless,
            name=f"{side}_hinge_knuckle",
        )
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.0, door_t / 2.0, door_h / 2.0)),
        material=dark_glass,
        name="door_panel",
    )
    door.visual(
        Box((glass_w, 0.012, glass_h)),
        origin=Origin(xyz=(0.0, door_t - 0.006, glass_center_z)),
        material=dark_glass,
        name="glass_panel",
    )
    door.visual(
        Box((glass_w - 0.030, 0.003, glass_h - 0.030)),
        origin=Origin(xyz=(0.0, door_t - 0.0015, glass_center_z + 0.008)),
        material=gloss_glass,
        name="glass_reflection",
    )
    post_size = (0.026, 0.018, 0.045)
    post_y = door_t + (post_size[1] / 2.0) - 0.003
    post_z = door_h - 0.049
    post_x = 0.21
    door.visual(
        Box(post_size),
        origin=Origin(xyz=(-post_x, post_y, post_z)),
        material=brushed_steel,
        name="handle_left_post",
    )
    door.visual(
        Box(post_size),
        origin=Origin(xyz=(post_x, post_y, post_z)),
        material=brushed_steel,
        name="handle_right_post",
    )
    handle_radius = 0.011
    door.visual(
        Cylinder(radius=handle_radius, length=0.46),
        origin=Origin(
            xyz=(0.0, post_y + (post_size[1] / 2.0) + handle_radius, post_z + 0.002),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t + 0.05, door_h)),
        mass=9.0,
        origin=Origin(xyz=(0.0, (door_t + 0.05) / 2.0, door_h / 2.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-(math.pi / 2.0),
            upper=0.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")

    front_left_frame = housing.get_visual("front_left_frame")
    front_right_frame = housing.get_visual("front_right_frame")
    bottom_sill = housing.get_visual("bottom_sill")
    top_fascia = housing.get_visual("top_fascia")
    display_window = housing.get_visual("display_window")
    cavity_back = housing.get_visual("cavity_back")
    left_hinge_pin = housing.get_visual("left_hinge_pin")
    right_hinge_pin = housing.get_visual("right_hinge_pin")

    left_hinge_knuckle = door.get_visual("left_hinge_knuckle")
    right_hinge_knuckle = door.get_visual("right_hinge_knuckle")
    door_panel = door.get_visual("door_panel")
    glass_panel = door.get_visual("glass_panel")
    handle_bar = door.get_visual("handle_bar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        housing,
        door,
        elem_a=left_hinge_pin,
        elem_b=left_hinge_knuckle,
        reason="The left hinge pin intentionally runs inside the left door knuckle.",
    )
    ctx.allow_overlap(
        housing,
        door,
        elem_a=right_hinge_pin,
        elem_b=right_hinge_knuckle,
        reason="The right hinge pin intentionally runs inside the right door knuckle.",
    )
    ctx.allow_overlap(
        housing,
        door,
        elem_a=bottom_sill,
        elem_b=left_hinge_knuckle,
        reason="The left hinge knuckle nests into the bottom sill hinge recess through the swing range.",
    )
    ctx.allow_overlap(
        housing,
        door,
        elem_a=bottom_sill,
        elem_b=right_hinge_knuckle,
        reason="The right hinge knuckle nests into the bottom sill hinge recess through the swing range.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    hinge_limits = door_hinge.motion_limits
    ctx.check(
        "door_hinge_axis_is_widthwise",
        door_hinge.axis == (1.0, 0.0, 0.0),
        details=f"Expected bottom hinge axis (1, 0, 0), got {door_hinge.axis!r}",
    )
    ctx.check(
        "door_hinge_limits_are_drop_down",
        hinge_limits is not None
        and hinge_limits.upper == 0.0
        and hinge_limits.lower is not None
        and hinge_limits.lower <= -1.4,
        details=f"Expected a drop-down door range terminating at 0 rad, got {hinge_limits!r}",
    )
    ctx.expect_gap(
        door,
        housing,
        axis="x",
        min_gap=0.001,
        max_gap=0.0035,
        positive_elem=door_panel,
        negative_elem=front_left_frame,
        name="door_fills_left_side_of_front_opening",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="x",
        min_gap=0.001,
        max_gap=0.0035,
        positive_elem=front_right_frame,
        negative_elem=door_panel,
        name="door_fills_right_side_of_front_opening",
    )
    ctx.expect_gap(
        door,
        housing,
        axis="z",
        min_gap=0.001,
        max_gap=0.0035,
        positive_elem=door_panel,
        negative_elem=bottom_sill,
        name="door_seats_on_bottom_hinge_line",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="z",
        min_gap=0.001,
        max_gap=0.0035,
        positive_elem=top_fascia,
        negative_elem=door_panel,
        name="door_closes_up_to_top_fascia",
    )
    ctx.expect_within(
        door,
        door,
        axes="xz",
        inner_elem=glass_panel,
        name="glass_panel_sits_within_door_face",
    )
    ctx.expect_within(
        door,
        door,
        axes="x",
        inner_elem=handle_bar,
        name="pull_handle_stays_within_door_width",
    )
    ctx.expect_overlap(
        door,
        door,
        axes="x",
        elem_a=handle_bar,
        elem_b=glass_panel,
        min_overlap=0.42,
        name="pull_handle_runs_across_door_top_span",
    )
    ctx.expect_gap(
        door,
        door,
        axis="y",
        min_gap=0.010,
        positive_elem=handle_bar,
        negative_elem=glass_panel,
        name="pull_handle_projects_proud_of_glass",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="z",
        min_gap=0.03,
        positive_elem=display_window,
        negative_elem=glass_panel,
        name="control_band_sits_above_door_glass",
    )

    if hinge_limits is not None and hinge_limits.lower is not None and hinge_limits.upper is not None:
        with ctx.pose({door_hinge: hinge_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_unintentional_overlap")
            ctx.fail_if_isolated_parts(name="door_closed_no_floating")

        with ctx.pose({door_hinge: hinge_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_unintentional_overlap")
            ctx.fail_if_isolated_parts(name="door_open_no_floating")
            ctx.expect_gap(
                door,
                housing,
                axis="y",
                min_gap=0.20,
                positive_elem=handle_bar,
                negative_elem=display_window,
                name="open_door_projects_forward_from_oven_face",
            )
            ctx.expect_gap(
                housing,
                door,
                axis="z",
                min_gap=0.40,
                positive_elem=display_window,
                negative_elem=handle_bar,
                name="open_door_drops_handle_below_control_band",
            )
            ctx.expect_gap(
                door,
                housing,
                axis="y",
                min_gap=0.30,
                positive_elem=glass_panel,
                negative_elem=cavity_back,
                name="open_door_reveals_oven_cavity",
            )

            open_aabb = ctx.part_world_aabb(door)
            if open_aabb is None:
                ctx.fail("door_open_pose_has_geometry", "Door world AABB was unavailable in the open pose.")
            else:
                open_y_span = open_aabb[1][1] - open_aabb[0][1]
                open_z_span = open_aabb[1][2] - open_aabb[0][2]
                ctx.check(
                    "open_door_reads_as_horizontal_drop_down_panel",
                    open_y_span > 0.34 and open_z_span < 0.15,
                    details=f"Expected a mostly horizontal open door, got y-span={open_y_span:.4f}, z-span={open_z_span:.4f}",
                )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
