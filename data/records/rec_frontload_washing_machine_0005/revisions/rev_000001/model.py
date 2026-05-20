from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

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

_SCRIPT_ROOT = Path(__file__).parent if "__file__" in globals() else Path("/tmp")
if not _SCRIPT_ROOT.is_absolute():
    _SCRIPT_ROOT = Path("/tmp")
ASSETS = AssetContext(_SCRIPT_ROOT)
HERE = ASSETS.asset_root


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_washer", assets=ASSETS)

    width = 0.60
    depth = 0.62
    height = 0.85
    wall = 0.02
    front_face = depth * 0.5
    door_center_z = 0.40
    door_outer_radius = 0.235
    body_opening_radius = 0.205
    drum_radius = 0.155
    drum_center_x = 0.02
    drum_length = 0.30
    door_hinge_x = front_face
    door_hinge_y = door_outer_radius + 0.012
    front_frame_thickness = 0.018

    body_charcoal = model.material("body_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    charcoal_trim = model.material("charcoal_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.38, 0.48, 0.53, 0.30))
    drum_steel = model.material("drum_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    def _add_ring_segments(
        part,
        *,
        radius: float,
        center_y: float,
        center_z: float,
        x_center: float,
        x_size: float,
        radial_size: float,
        count: int,
        material,
        base_name: str,
        overlap_scale: float = 1.04,
    ) -> None:
        tangent = 2.0 * math.pi * radius / count * overlap_scale
        for index in range(count):
            angle = 2.0 * math.pi * index / count
            part.visual(
                Box((x_size, tangent, radial_size)),
                origin=Origin(
                    xyz=(
                        x_center,
                        center_y + radius * math.cos(angle),
                        center_z + radius * math.sin(angle),
                    ),
                    rpy=(angle + math.pi * 0.5, 0.0, 0.0),
                ),
                material=material,
                name=base_name if index == 0 else f"{base_name}_{index}",
            )

    def _add_drum_slats(
        part,
        *,
        radius: float,
        length: float,
        width: float,
        thickness: float,
        count: int,
        material,
        base_name: str,
    ) -> None:
        for index in range(count):
            angle = 2.0 * math.pi * index / count
            part.visual(
                Box((length, width, thickness)),
                origin=Origin(
                    xyz=(0.0, radius * math.cos(angle), radius * math.sin(angle)),
                    rpy=(angle - math.pi * 0.5, 0.0, 0.0),
                ),
                material=material,
                name=base_name if index == 0 else f"{base_name}_{index}",
            )

    body = model.part("body")
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, (width - wall) * 0.5, height * 0.5)),
        material=body_charcoal,
        name="left_side",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, -(width - wall) * 0.5, height * 0.5)),
        material=body_charcoal,
        name="right_side",
    )
    body.visual(
        Box((depth, width - 2.0 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall * 0.5)),
        material=body_charcoal,
        name="top_panel",
    )
    body.visual(
        Box((depth - 0.02, width - 2.0 * wall, wall)),
        origin=Origin(xyz=(-0.01, 0.0, wall * 0.5)),
        material=body_charcoal,
        name="base_panel",
    )
    body.visual(
        Box((wall, width - 2.0 * wall, height - 2.0 * wall)),
        origin=Origin(xyz=(-depth * 0.5 + wall * 0.5, 0.0, height * 0.5)),
        material=body_charcoal,
        name="rear_panel",
    )
    body.visual(
        Box((front_frame_thickness, 0.070, 0.500)),
        origin=Origin(xyz=(front_face - front_frame_thickness * 0.5, 0.265, door_center_z)),
        material=body_charcoal,
        name="front_left_frame",
    )
    body.visual(
        Box((front_frame_thickness, 0.070, 0.500)),
        origin=Origin(xyz=(front_face - front_frame_thickness * 0.5, -0.265, door_center_z)),
        material=body_charcoal,
        name="front_right_frame",
    )
    body.visual(
        Box((front_frame_thickness, 0.460, 0.090)),
        origin=Origin(xyz=(front_face - front_frame_thickness * 0.5, 0.0, 0.660)),
        material=body_charcoal,
        name="front_top_frame",
    )
    body.visual(
        Box((front_frame_thickness, 0.460, 0.120)),
        origin=Origin(xyz=(front_face - front_frame_thickness * 0.5, 0.0, 0.115)),
        material=body_charcoal,
        name="front_bottom_frame",
    )
    _add_ring_segments(
        body,
        radius=0.219,
        center_y=0.0,
        center_z=door_center_z,
        x_center=front_face - 0.026,
        x_size=0.026,
        radial_size=0.020,
        count=30,
        material=rubber,
        base_name="door_boot",
    )
    body.visual(
        Box((0.012, 0.46, 0.11)),
        origin=Origin(xyz=(front_face + 0.006, 0.0, 0.735)),
        material=charcoal_trim,
        name="control_panel",
    )
    body.visual(
        Box((0.004, 0.22, 0.046)),
        origin=Origin(xyz=(front_face + 0.014, -0.105, 0.735)),
        material=dark_glass,
        name="screen",
    )
    body.visual(
        Box((0.016, 0.030, 0.090)),
        origin=Origin(xyz=(front_face - 0.008, 0.238, door_center_z + 0.075)),
        material=body_charcoal,
        name="hinge_upper_support",
    )
    body.visual(
        Box((0.016, 0.030, 0.090)),
        origin=Origin(xyz=(front_face - 0.008, 0.238, door_center_z - 0.075)),
        material=body_charcoal,
        name="hinge_lower_support",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.220),
        origin=Origin(xyz=(front_face, door_hinge_y, door_center_z)),
        material=satin_steel,
        name="hinge_pin",
    )
    _add_ring_segments(
        body,
        radius=0.031,
        center_y=0.0,
        center_z=door_center_z,
        x_center=-0.2875,
        x_size=0.030,
        radial_size=0.018,
        count=16,
        material=charcoal_trim,
        base_name="bearing_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((depth, width, height)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    drum = model.part("drum")
    _add_ring_segments(
        drum,
        radius=drum_radius,
        center_y=0.0,
        center_z=0.0,
        x_center=-0.125,
        x_size=0.016,
        radial_size=0.012,
        count=24,
        material=drum_steel,
        base_name="rear_hoop",
    )
    _add_ring_segments(
        drum,
        radius=drum_radius,
        center_y=0.0,
        center_z=0.0,
        x_center=0.0,
        x_size=0.016,
        radial_size=0.012,
        count=24,
        material=drum_steel,
        base_name="mid_hoop",
    )
    _add_ring_segments(
        drum,
        radius=0.166,
        center_y=0.0,
        center_z=0.0,
        x_center=drum_length * 0.5 - 0.01,
        x_size=0.020,
        radial_size=0.018,
        count=28,
        material=satin_steel,
        base_name="front_rim",
    )
    _add_drum_slats(
        drum,
        radius=drum_radius,
        length=drum_length - 0.02,
        width=0.024,
        thickness=0.008,
        count=14,
        material=drum_steel,
        base_name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.159, length=0.020),
        origin=Origin(
            xyz=(-0.140, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=drum_steel,
        name="rear_wall",
    )
    for index, angle in enumerate((0.0, math.tau / 3.0, 2.0 * math.tau / 3.0)):
        drum.visual(
            Box((0.230, 0.050, 0.026)),
            origin=Origin(
                xyz=(0.0, 0.142 * math.cos(angle), 0.142 * math.sin(angle)),
                rpy=(angle - math.pi * 0.5, 0.0, 0.0),
            ),
            material=drum_steel,
            name="drum_paddle" if index == 0 else f"drum_paddle_{index}",
        )
    drum.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(
            xyz=(-0.150, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=drum_steel,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.022, length=0.135),
        origin=Origin(
            xyz=(-0.225, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=satin_steel,
        name="axle_stub",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_radius, length=drum_length),
        mass=8.0,
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
    )

    door = model.part("door")
    _add_ring_segments(
        door,
        radius=0.210,
        center_y=-door_hinge_y,
        center_z=0.0,
        x_center=0.022,
        x_size=0.028,
        radial_size=0.028,
        count=32,
        material=satin_steel,
        base_name="bezel_ring",
    )
    _add_ring_segments(
        door,
        radius=0.188,
        center_y=-door_hinge_y,
        center_z=0.0,
        x_center=0.014,
        x_size=0.022,
        radial_size=0.018,
        count=28,
        material=charcoal_trim,
        base_name="inner_ring",
    )
    door.visual(
        Box((0.012, 0.038, 0.280)),
        origin=Origin(xyz=(0.018, -0.018, 0.0)),
        material=charcoal_trim,
        name="hinge_spine",
    )
    door.visual(
        Cylinder(radius=0.004, length=0.220),
        origin=Origin(xyz=(0.0081, 0.0, 0.0)),
        material=charcoal_trim,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.160, length=0.008),
        origin=Origin(
            xyz=(0.018, -door_hinge_y, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_glass,
        name="glass_window",
    )
    door.visual(
        Box((0.032, 0.038, 0.110)),
        origin=Origin(xyz=(0.030, -0.402, 0.0)),
        material=charcoal_trim,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.05, door_outer_radius * 2.0, door_outer_radius * 2.0)),
        mass=3.2,
        origin=Origin(xyz=(0.018, -door_hinge_y, 0.0)),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.037, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_steel,
        name="dial_knob",
    )
    selector_dial.visual(
        Box((0.010, 0.008, 0.018)),
        origin=Origin(xyz=(0.011, 0.0, 0.026)),
        material=charcoal_trim,
        name="dial_pointer",
    )
    selector_dial.inertial = Inertial.from_geometry(
        Box((0.016, 0.080, 0.080)),
        mass=0.18,
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drum,
        origin=Origin(xyz=(drum_center_x, 0.0, door_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=8.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "body_to_selector_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(front_face + 0.012, 0.165, 0.735)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-2.6,
            upper=2.6,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, seed=0)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    selector_dial = object_model.get_part("selector_dial")
    drum_axle = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("body_to_selector_dial")

    door_boot = body.get_visual("door_boot")
    bearing_mount = body.get_visual("bearing_mount")
    front_top_frame = body.get_visual("front_top_frame")
    control_panel = body.get_visual("control_panel")
    hinge_pin = body.get_visual("hinge_pin")
    bezel_ring = door.get_visual("bezel_ring")
    glass_window = door.get_visual("glass_window")
    hinge_barrel = door.get_visual("hinge_barrel")
    front_rim = drum.get_visual("front_rim")
    axle_stub = drum.get_visual("axle_stub")
    drum_paddle = drum.get_visual("drum_paddle")
    dial_knob = selector_dial.get_visual("dial_knob")
    dial_pointer = selector_dial.get_visual("dial_pointer")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=64,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_aabb_available", "Body world AABB could not be resolved.")
    else:
        body_depth_measured = body_aabb[1][0] - body_aabb[0][0]
        body_width_measured = body_aabb[1][1] - body_aabb[0][1]
        body_height_measured = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "washer_body_scale",
            0.58 <= body_width_measured <= 0.62
            and 0.60 <= body_depth_measured <= 0.65
            and 0.82 <= body_height_measured <= 0.88,
            (
                f"Measured body dims were "
                f"{body_depth_measured:.3f} x {body_width_measured:.3f} x {body_height_measured:.3f} m."
            ),
        )

    ctx.check(
        "drum_axle_axis",
        tuple(drum_axle.axis) == (1.0, 0.0, 0.0),
        f"Drum axis was {drum_axle.axis!r}, expected rotation about +X.",
    )
    ctx.check(
        "door_hinge_axis",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        f"Door hinge axis was {door_hinge.axis!r}, expected vertical +Z.",
    )
    ctx.check(
        "selector_dial_axis",
        tuple(dial_joint.axis) == (1.0, 0.0, 0.0),
        f"Selector dial axis was {dial_joint.axis!r}, expected front-panel normal +X.",
    )

    ctx.expect_overlap(door, drum, axes="yz", min_overlap=0.20)
    ctx.expect_overlap(door, body, axes="yz", min_overlap=0.34)
    ctx.expect_gap(
        door,
        body,
        axis="x",
        min_gap=0.018,
        max_gap=0.050,
        positive_elem=bezel_ring,
        negative_elem=door_boot,
    )
    ctx.expect_gap(
        door,
        drum,
        axis="x",
        min_gap=0.10,
        max_gap=0.18,
        positive_elem=glass_window,
        negative_elem=front_rim,
    )
    ctx.expect_contact(door, body, elem_a=hinge_barrel, elem_b=hinge_pin, contact_tol=0.0002)
    ctx.expect_contact(drum, body, elem_a=axle_stub, elem_b=bearing_mount, contact_tol=0.0001)
    ctx.expect_contact(selector_dial, body, elem_a=dial_knob, elem_b=control_panel)
    ctx.expect_overlap(door, drum, axes="yz", min_overlap=0.22, elem_a=glass_window)

    glass_rest = ctx.part_element_world_aabb(door, elem=glass_window)
    paddle_rest = ctx.part_element_world_aabb(drum, elem=drum_paddle)
    pointer_rest = ctx.part_element_world_aabb(selector_dial, elem=dial_pointer)

    with ctx.pose({door_hinge: 1.30}):
        glass_open = ctx.part_element_world_aabb(door, elem=glass_window)
        if glass_rest is None or glass_open is None:
            ctx.fail("door_window_pose_probe", "Door glass AABB was unavailable in one of the tested poses.")
        else:
            ctx.check(
                "door_swings_forward",
                glass_open[0][0] > glass_rest[0][0] + 0.05,
                (
                    f"Door glass min X moved from {glass_rest[0][0]:.3f} to "
                    f"{glass_open[0][0]:.3f}, which is not a clear forward swing."
                ),
            )
        ctx.expect_gap(
            door,
            body,
            axis="x",
            min_gap=0.08,
            positive_elem=glass_window,
            negative_elem=front_top_frame,
        )
        ctx.expect_contact(door, body, elem_a=hinge_barrel, elem_b=hinge_pin, contact_tol=0.0002)

    with ctx.pose({drum_axle: math.pi * 0.5}):
        paddle_turn = ctx.part_element_world_aabb(drum, elem=drum_paddle)
        if paddle_rest is None or paddle_turn is None:
            ctx.fail("drum_paddle_pose_probe", "Drum paddle AABB was unavailable in one of the tested poses.")
        else:
            rest_center_z = 0.5 * (paddle_rest[0][2] + paddle_rest[1][2])
            turn_center_z = 0.5 * (paddle_turn[0][2] + paddle_turn[1][2])
            ctx.check(
                "drum_rotates_on_axle",
                abs(turn_center_z - rest_center_z) > 0.07,
                (
                    f"Drum paddle center Z only changed from {rest_center_z:.3f} to "
                    f"{turn_center_z:.3f}."
                ),
            )
        ctx.expect_contact(drum, body, elem_a=axle_stub, elem_b=bearing_mount, contact_tol=0.0001)
        ctx.expect_overlap(drum, body, axes="yz", min_overlap=0.28)

    with ctx.pose({dial_joint: 1.25}):
        pointer_turn = ctx.part_element_world_aabb(selector_dial, elem=dial_pointer)
        if pointer_rest is None or pointer_turn is None:
            ctx.fail("dial_pointer_pose_probe", "Dial pointer AABB was unavailable in one of the tested poses.")
        else:
            rest_center_y = 0.5 * (pointer_rest[0][1] + pointer_rest[1][1])
            turn_center_y = 0.5 * (pointer_turn[0][1] + pointer_turn[1][1])
            ctx.check(
                "selector_dial_turns",
                abs(turn_center_y - rest_center_y) > 0.012,
                (
                    f"Dial pointer center Y only changed from {rest_center_y:.3f} to "
                    f"{turn_center_y:.3f}."
                ),
            )
        ctx.expect_contact(selector_dial, body, elem_a=dial_knob, elem_b=control_panel)

    for articulation, label in (
        (door_hinge, "door_hinge"),
        (drum_axle, "drum_axle"),
        (dial_joint, "selector_dial"),
    ):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_lower_no_floating")
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
