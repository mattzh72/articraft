from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path
import sys

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


def _script_root() -> Path:
    candidates: list[str | None] = [
        getattr(globals().get("__spec__"), "origin", None),
        globals().get("__file__"),
    ]
    loader = globals().get("__loader__")
    if loader is not None and hasattr(loader, "get_filename"):
        try:
            candidates.append(loader.get_filename())
        except Exception:
            pass
    candidates.append(sys.argv[0] if sys.argv else None)

    for raw in candidates:
        if raw and Path(raw).is_absolute():
            return Path(raw).parent

    for raw in candidates:
        if raw and raw not in ("", "-c", "<stdin>"):
            path = Path(raw)
            if path.parts and path.parent != Path():
                return path.parent

    return Path("/tmp")


ASSETS = AssetContext(_script_root())


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven", assets=ASSETS)

    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.16, 0.18, 1.0))
    oven_glass = model.material("oven_glass", rgba=(0.55, 0.66, 0.74, 0.28))
    rack_dark = model.material("rack_dark", rgba=(0.22, 0.22, 0.24, 1.0))
    element_hot = model.material("element_hot", rgba=(0.52, 0.33, 0.16, 1.0))

    body_width = 0.52
    body_depth = 0.35
    body_height = 0.32
    shell_t = 0.012
    foot_h = 0.016

    door_width = 0.36
    door_height = 0.22
    door_t = 0.022
    side_margin = (body_width - door_width) / 2.0
    header_h = body_height - door_height - 0.060
    hinge_z = body_height - header_h - door_height
    front_y = -body_depth / 2.0
    front_frame_y = front_y + shell_t / 2.0
    door_closed_y = front_y - door_t / 2.0 - 0.001

    pin_r = 0.0045
    pin_len = 0.012
    boss_r = 0.006
    boss_len = 0.004

    body = model.part("body")
    body.visual(
        Box((shell_t, body_depth, body_height - foot_h)),
        origin=Origin(xyz=(-body_width / 2.0 + shell_t / 2.0, 0.0, foot_h + (body_height - foot_h) / 2.0)),
        material=chrome,
        name="left_wall",
    )
    body.visual(
        Box((shell_t, body_depth, body_height - foot_h)),
        origin=Origin(xyz=(body_width / 2.0 - shell_t / 2.0, 0.0, foot_h + (body_height - foot_h) / 2.0)),
        material=chrome,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * shell_t, body_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + shell_t / 2.0)),
        material=chrome,
        name="bottom_panel",
    )
    body.visual(
        Box((body_width - 2.0 * shell_t, body_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_height - shell_t / 2.0)),
        material=chrome,
        name="top_panel",
    )
    body.visual(
        Box((body_width - 2.0 * shell_t, shell_t, body_height - foot_h - 2.0 * shell_t)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - shell_t / 2.0,
                foot_h + shell_t + (body_height - foot_h - 2.0 * shell_t) / 2.0,
            )
        ),
        material=chrome,
        name="back_panel",
    )
    body.visual(
        Box((side_margin, shell_t, door_height)),
        origin=Origin(
            xyz=(
                -(door_width / 2.0 + side_margin / 2.0),
                front_frame_y,
                hinge_z + door_height / 2.0,
            )
        ),
        material=chrome,
        name="left_jamb",
    )
    body.visual(
        Box((side_margin, shell_t, door_height)),
        origin=Origin(
            xyz=(
                door_width / 2.0 + side_margin / 2.0,
                front_frame_y,
                hinge_z + door_height / 2.0,
            )
        ),
        material=chrome,
        name="right_jamb",
    )
    body.visual(
        Box((body_width - 2.0 * shell_t, shell_t, header_h)),
        origin=Origin(xyz=(0.0, front_frame_y, body_height - header_h / 2.0)),
        material=chrome,
        name="front_header",
    )
    body.visual(
        Box((body_width - 2.0 * shell_t, shell_t, hinge_z - foot_h - shell_t)),
        origin=Origin(
            xyz=(
                0.0,
                front_frame_y,
                foot_h + shell_t + (hinge_z - foot_h - shell_t) / 2.0,
            )
        ),
        material=chrome,
        name="front_sill",
    )

    foot_size = (0.044, 0.044, foot_h)
    for name, x_sign, y_sign in (
        ("front_left_foot", -1.0, -1.0),
        ("front_right_foot", 1.0, -1.0),
        ("rear_left_foot", -1.0, 1.0),
        ("rear_right_foot", 1.0, 1.0),
    ):
        body.visual(
            Box(foot_size),
            origin=Origin(
                xyz=(
                    x_sign * (body_width / 2.0 - 0.065),
                    y_sign * (body_depth / 2.0 - 0.065),
                    foot_h / 2.0,
                )
            ),
            material=dark_trim,
            name=name,
        )

    body.visual(
        Box((body_width - 2.0 * shell_t, 0.23, 0.004)),
        origin=Origin(xyz=(0.0, 0.015, 0.118)),
        material=rack_dark,
        name="rack",
    )
    body.visual(
        Cylinder(radius=0.003, length=body_width - 2.0 * shell_t),
        origin=Origin(xyz=(0.0, -0.050, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=element_hot,
        name="lower_element_front",
    )
    body.visual(
        Cylinder(radius=0.003, length=body_width - 2.0 * shell_t),
        origin=Origin(xyz=(0.0, 0.050, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=element_hot,
        name="lower_element_rear",
    )
    body.visual(
        Cylinder(radius=0.003, length=body_width - 2.0 * shell_t),
        origin=Origin(xyz=(0.0, -0.050, 0.234), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=element_hot,
        name="upper_element_front",
    )
    body.visual(
        Cylinder(radius=0.003, length=body_width - 2.0 * shell_t),
        origin=Origin(xyz=(0.0, 0.050, 0.234), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=element_hot,
        name="upper_element_rear",
    )

    left_boss_x = -(door_width / 2.0 + pin_len + boss_len / 2.0)
    right_boss_x = door_width / 2.0 + pin_len + boss_len / 2.0
    body.visual(
        Cylinder(radius=boss_r, length=boss_len),
        origin=Origin(xyz=(left_boss_x, door_closed_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="left_hinge_boss",
    )
    body.visual(
        Cylinder(radius=boss_r, length=boss_len),
        origin=Origin(xyz=(right_boss_x, door_closed_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="right_hinge_boss",
    )
    body.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(left_boss_x, -0.180, hinge_z)),
        material=dark_trim,
        name="left_hinge_support",
    )
    body.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(right_boss_x, -0.180, hinge_z)),
        material=dark_trim,
        name="right_hinge_support",
    )
    body.visual(
        Box((side_margin * 0.72, shell_t * 0.5, door_height - 0.018)),
        origin=Origin(
            xyz=(
                door_width / 2.0 + side_margin / 2.0,
                front_y - shell_t / 4.0,
                hinge_z + door_height / 2.0,
            )
        ),
        material=dark_trim,
        name="control_panel",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    door = model.part("door")
    rail_w = 0.024
    top_rail_h = 0.032
    bottom_rail_h = 0.026
    door.visual(
        Box((rail_w, door_t, door_height)),
        origin=Origin(xyz=(-(door_width / 2.0 - rail_w / 2.0), 0.0, door_height / 2.0)),
        material=chrome,
        name="left_rail",
    )
    door.visual(
        Box((rail_w, door_t, door_height)),
        origin=Origin(xyz=((door_width / 2.0 - rail_w / 2.0), 0.0, door_height / 2.0)),
        material=chrome,
        name="right_rail",
    )
    door.visual(
        Box((door_width, door_t, top_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, door_height - top_rail_h / 2.0)),
        material=chrome,
        name="top_rail",
    )
    door.visual(
        Box((door_width, door_t, bottom_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, bottom_rail_h / 2.0)),
        material=chrome,
        name="bottom_rail",
    )
    glass_w = door_width - 2.0 * rail_w
    glass_h = door_height - top_rail_h - bottom_rail_h
    door.visual(
        Box((glass_w, 0.008, glass_h)),
        origin=Origin(xyz=(0.0, 0.003, bottom_rail_h + glass_h / 2.0)),
        material=oven_glass,
        name="glass",
    )
    door.visual(
        Box((0.014, 0.010, 0.024)),
        origin=Origin(xyz=(-0.084, -0.016, door_height - top_rail_h - 0.012)),
        material=chrome,
        name="left_handle_post",
    )
    door.visual(
        Box((0.014, 0.010, 0.024)),
        origin=Origin(xyz=(0.084, -0.016, door_height - top_rail_h - 0.012)),
        material=chrome,
        name="right_handle_post",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.240),
        origin=Origin(xyz=(0.0, -0.023, door_height - top_rail_h - 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="handle",
    )
    left_pin_x = -(door_width / 2.0 + pin_len / 2.0)
    right_pin_x = door_width / 2.0 + pin_len / 2.0
    door.visual(
        Cylinder(radius=pin_r, length=pin_len),
        origin=Origin(xyz=(left_pin_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="left_pin",
    )
    door.visual(
        Cylinder(radius=pin_r, length=pin_len),
        origin=Origin(xyz=(right_pin_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="right_pin",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_t, door_height)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, door_height / 2.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, door_closed_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    knob_radius = 0.024
    knob_depth = 0.018
    knob_center_x = door_width / 2.0 + side_margin / 2.0
    knob_center_y = front_y - shell_t / 2.0 - knob_depth / 2.0
    knob_z_positions = {
        "function_knob": 0.246,
        "temperature_knob": 0.178,
        "timer_knob": 0.110,
    }
    for knob_name, knob_z in knob_z_positions.items():
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=knob_radius, length=knob_depth),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="knob_body",
        )
        knob.visual(
            Box((0.006, 0.003, 0.014)),
            origin=Origin(xyz=(0.0, -knob_depth / 2.0 - 0.0015, 0.011)),
            material=dark_trim,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((knob_radius * 2.0, knob_depth, knob_radius * 2.0)),
            mass=0.08,
            origin=Origin(),
        )
        model.articulation(
            f"{knob_name}_joint",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_center_x, knob_center_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-2.35, upper=2.35),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")
    function_knob = object_model.get_part("function_knob")
    temperature_knob = object_model.get_part("temperature_knob")
    timer_knob = object_model.get_part("timer_knob")
    function_knob_joint = object_model.get_articulation("function_knob_joint")
    temperature_knob_joint = object_model.get_articulation("temperature_knob_joint")
    timer_knob_joint = object_model.get_articulation("timer_knob_joint")

    front_header = body.get_visual("front_header")
    front_sill = body.get_visual("front_sill")
    left_hinge_boss = body.get_visual("left_hinge_boss")
    right_hinge_boss = body.get_visual("right_hinge_boss")
    control_panel = body.get_visual("control_panel")

    top_rail = door.get_visual("top_rail")
    bottom_rail = door.get_visual("bottom_rail")
    glass = door.get_visual("glass")
    handle = door.get_visual("handle")
    left_pin = door.get_visual("left_pin")
    right_pin = door.get_visual("right_pin")
    function_knob_body = function_knob.get_visual("knob_body")
    function_indicator = function_knob.get_visual("indicator")
    temperature_knob_body = temperature_knob.get_visual("knob_body")
    timer_knob_body = timer_knob.get_visual("knob_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.07)
    ctx.expect_gap(
        body,
        door,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=front_header,
        negative_elem=top_rail,
        name="door_top_seats_flush_with_header",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=front_sill,
        negative_elem=bottom_rail,
        name="door_bottom_seats_flush_with_sill",
    )
    ctx.expect_contact(door, body, elem_a=left_pin, elem_b=left_hinge_boss)
    ctx.expect_contact(door, body, elem_a=right_pin, elem_b=right_hinge_boss)
    ctx.expect_gap(
        door,
        door,
        axis="y",
        min_gap=0.010,
        positive_elem=glass,
        negative_elem=handle,
        name="handle_sits_proud_of_glass",
    )
    ctx.expect_contact(function_knob, body, elem_a=function_knob_body, elem_b=control_panel)
    ctx.expect_contact(temperature_knob, body, elem_a=temperature_knob_body, elem_b=control_panel)
    ctx.expect_contact(timer_knob, body, elem_a=timer_knob_body, elem_b=control_panel)
    ctx.expect_gap(
        body,
        function_knob,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=control_panel,
        negative_elem=function_knob_body,
        name="function_knob_mounts_flush",
    )
    ctx.expect_origin_gap(
        function_knob,
        temperature_knob,
        axis="z",
        min_gap=0.05,
        max_gap=0.08,
        name="upper_knob_spacing",
    )
    ctx.expect_origin_gap(
        temperature_knob,
        timer_knob,
        axis="z",
        min_gap=0.05,
        max_gap=0.08,
        name="lower_knob_spacing",
    )
    ctx.expect_gap(
        function_knob,
        door,
        axis="x",
        min_gap=0.02,
        positive_elem=function_knob_body,
        negative_elem=glass,
        name="control_bank_sits_to_door_right",
    )
    with ctx.pose({door_hinge: 1.35}):
        ctx.expect_overlap(door, body, axes="x", min_overlap=0.35)
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=0.16,
            positive_elem=front_header,
            negative_elem=top_rail,
            name="open_door_swings_forward",
        )
        ctx.expect_contact(door, body, elem_a=left_pin, elem_b=left_hinge_boss)
        ctx.expect_contact(door, body, elem_a=right_pin, elem_b=right_hinge_boss)
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
    for joint in (door_hinge, function_knob_joint, temperature_knob_joint, timer_knob_joint):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
    with ctx.pose({function_knob_joint: 1.7}):
        ctx.expect_contact(function_knob, body, elem_a=function_knob_body, elem_b=control_panel)
        ctx.expect_gap(
            body,
            function_knob,
            axis="y",
            min_gap=0.009,
            max_gap=0.03,
            positive_elem=control_panel,
            negative_elem=function_indicator,
            name="function_indicator_stays_forward_of_panel",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
