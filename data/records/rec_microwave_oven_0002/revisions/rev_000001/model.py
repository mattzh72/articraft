from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="otr_microwave_oven", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    stainless_dark = model.material("stainless_dark", rgba=(0.58, 0.60, 0.62, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.21, 0.23, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.13, 0.16, 0.55))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.21, 0.23, 0.60))

    width = 0.76
    depth = 0.42
    height = 0.43
    shell_t = 0.012
    front_face_t = 0.022

    door_width = 0.59
    door_height = 0.360
    door_thickness = 0.022
    door_left_x = -0.375
    door_bottom_z = 0.046
    door_center_y = depth / 2 + 0.022

    control_left_x = door_left_x + door_width + 0.005
    control_width = width / 2 - control_left_x
    control_center_x = control_left_x + control_width / 2

    cavity_center_x = (-width / 2 + shell_t + control_left_x) / 2
    cavity_floor_z = shell_t

    body = model.part("body")
    body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_t / 2)),
        material=charcoal,
        name="top_shell",
    )
    body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t / 2)),
        material=charcoal,
        name="bottom_shell",
    )
    body.visual(
        Box((shell_t, depth, height - 2 * shell_t)),
        origin=Origin(xyz=(-width / 2 + shell_t / 2, 0.0, height / 2)),
        material=charcoal,
        name="left_shell",
    )
    body.visual(
        Box((shell_t, depth, height - 2 * shell_t)),
        origin=Origin(xyz=(width / 2 - shell_t / 2, 0.0, height / 2)),
        material=charcoal,
        name="right_shell",
    )
    body.visual(
        Box((width - 2 * shell_t, shell_t, height - 2 * shell_t)),
        origin=Origin(xyz=(0.0, -depth / 2 + shell_t / 2, height / 2)),
        material=charcoal,
        name="back_shell",
    )
    body.visual(
        Box((shell_t, depth - 2 * shell_t, height - 2 * shell_t)),
        origin=Origin(xyz=(control_left_x + shell_t / 2, 0.0, height / 2)),
        material=charcoal,
        name="partition_wall",
    )
    body.visual(
        Box((control_width, front_face_t, height - 2 * shell_t)),
        origin=Origin(xyz=(control_center_x, depth / 2 + 0.001, height / 2)),
        material=stainless,
        name="control_frame",
    )
    body.visual(
        Box((control_width - 0.034, 0.010, height - 0.120)),
        origin=Origin(xyz=(control_center_x, depth / 2 - 0.001, height / 2 + 0.010)),
        material=glass,
        name="control_inset",
    )
    body.visual(
        Box((width - 0.024, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, depth / 2 - 0.005, 0.022)),
        material=charcoal,
        name="vent_backing",
    )
    for index, z_center in enumerate((0.014, 0.019, 0.024, 0.029, 0.034)):
        body.visual(
            Box((width - 0.040, 0.003, 0.0025)),
            origin=Origin(xyz=(0.0, depth / 2 + 0.010, z_center)),
            material=stainless_dark,
            name=f"vent_slat_{index}",
        )
    body.visual(
        Box((0.005, 0.012, door_height)),
        origin=Origin(
            xyz=(
                door_left_x - 0.0025,
                depth / 2 + 0.006,
                door_bottom_z + door_height / 2,
            )
        ),
        material=stainless_dark,
        name="hinge_strip",
    )
    body.visual(
        Box((0.005, 0.030, 0.055)),
        origin=Origin(
            xyz=(
                door_left_x - 0.0025,
                depth / 2 - 0.001,
                door_bottom_z + door_height - 0.032,
            )
        ),
        material=stainless_dark,
        name="top_hinge_leaf",
    )
    body.visual(
        Box((0.005, 0.030, 0.055)),
        origin=Origin(
            xyz=(
                door_left_x - 0.0025,
                depth / 2 - 0.001,
                door_bottom_z + 0.032,
            )
        ),
        material=stainless_dark,
        name="bottom_hinge_leaf",
    )
    body.visual(
        Box((width + 0.012, depth + 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, height + 0.002)),
        material=stainless,
        name="top_mount_plate",
    )
    body.visual(
        Box((0.006, depth * 0.78, 0.040)),
        origin=Origin(xyz=(-width / 2 + 0.003, 0.0, height + 0.020)),
        material=stainless_dark,
        name="left_mount_flange",
    )
    body.visual(
        Box((0.006, depth * 0.78, 0.040)),
        origin=Origin(xyz=(width / 2 - 0.003, 0.0, height + 0.020)),
        material=stainless_dark,
        name="right_mount_flange",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(cavity_center_x, 0.0, cavity_floor_z + 0.006)),
        material=charcoal,
        name="hub_post",
    )
    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2)),
    )

    door = model.part("door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2, -door_thickness / 2, door_height / 2)),
        material=stainless,
        name="outer_frame",
    )
    door.visual(
        Box((0.430, 0.006, 0.228)),
        origin=Origin(xyz=(0.285, -0.016, 0.208)),
        material=smoked_glass,
        name="window_glass",
    )
    door.visual(
        Box((0.454, 0.002, 0.010)),
        origin=Origin(xyz=(0.285, -0.001, 0.328)),
        material=stainless_dark,
        name="window_bead_top",
    )
    door.visual(
        Box((0.454, 0.002, 0.010)),
        origin=Origin(xyz=(0.285, -0.001, 0.088)),
        material=stainless_dark,
        name="window_bead_bottom",
    )
    door.visual(
        Box((0.010, 0.002, 0.236)),
        origin=Origin(xyz=(0.053, -0.001, 0.208)),
        material=stainless_dark,
        name="window_bead_left",
    )
    door.visual(
        Box((0.010, 0.002, 0.236)),
        origin=Origin(xyz=(0.517, -0.001, 0.208)),
        material=stainless_dark,
        name="window_bead_right",
    )
    door.visual(
        Cylinder(radius=0.0075, length=0.095),
        origin=Origin(
            xyz=(0.515, 0.016, 0.180),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=stainless_dark,
        name="handle_bar",
    )
    door.visual(
        Box((0.014, 0.018, 0.018)),
        origin=Origin(xyz=(0.472, 0.009, 0.180)),
        material=stainless_dark,
        name="handle_mount_left",
    )
    door.visual(
        Box((0.014, 0.018, 0.018)),
        origin=Origin(xyz=(0.558, 0.009, 0.180)),
        material=stainless_dark,
        name="handle_mount_right",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=6.0,
        origin=Origin(xyz=(door_width / 2, -door_thickness / 2, door_height / 2)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.133, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=glass,
        name="glass_disk",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.133, length=0.008),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_left_x, door_center_y, door_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "body_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turntable,
        origin=Origin(xyz=(cavity_center_x, 0.0, cavity_floor_z + 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    door_hinge = object_model.get_articulation("body_to_door")
    turntable_joint = object_model.get_articulation("body_to_turntable")
    top_shell = body.get_visual("top_shell")
    bottom_shell = body.get_visual("bottom_shell")
    control_frame = body.get_visual("control_frame")
    control_inset = body.get_visual("control_inset")
    vent_backing = body.get_visual("vent_backing")
    hinge_strip = body.get_visual("hinge_strip")
    top_hinge_leaf = body.get_visual("top_hinge_leaf")
    bottom_hinge_leaf = body.get_visual("bottom_hinge_leaf")
    top_mount_plate = body.get_visual("top_mount_plate")
    left_mount_flange = body.get_visual("left_mount_flange")
    right_mount_flange = body.get_visual("right_mount_flange")
    hub_post = body.get_visual("hub_post")
    outer_frame = door.get_visual("outer_frame")
    window_glass = door.get_visual("window_glass")
    handle_bar = door.get_visual("handle_bar")
    glass_disk = turntable.get_visual("glass_disk")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_within(door, door, axes="xyz", inner_elem=window_glass, outer_elem=outer_frame)
    ctx.expect_within(body, body, axes="xyz", inner_elem=control_inset, outer_elem=control_frame)
    ctx.expect_overlap(door, body, axes="z", elem_a=outer_frame, elem_b=control_frame, min_overlap=0.30)
    ctx.expect_gap(
        body,
        door,
        axis="x",
        positive_elem=control_frame,
        negative_elem=outer_frame,
        max_gap=0.02,
        max_penetration=0.0,
    )
    ctx.expect_overlap(body, door, axes="x", elem_a=vent_backing, elem_b=outer_frame, min_overlap=0.45)
    ctx.expect_gap(
        door,
        body,
        axis="z",
        positive_elem=outer_frame,
        negative_elem=vent_backing,
        min_gap=0.008,
        max_gap=0.020,
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem=top_mount_plate,
        negative_elem=top_shell,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_contact(body, body, elem_a=left_mount_flange, elem_b=top_mount_plate)
    ctx.expect_contact(body, body, elem_a=right_mount_flange, elem_b=top_mount_plate)
    ctx.expect_contact(door, body, elem_a=outer_frame, elem_b=hinge_strip)
    ctx.expect_overlap(door, body, axes="z", elem_a=outer_frame, elem_b=top_hinge_leaf, min_overlap=0.04)
    ctx.expect_overlap(door, body, axes="z", elem_a=outer_frame, elem_b=bottom_hinge_leaf, min_overlap=0.04)
    ctx.expect_overlap(door, door, axes="z", elem_a=handle_bar, elem_b=outer_frame, min_overlap=0.014)
    ctx.expect_contact(turntable, body, elem_a=glass_disk, elem_b=hub_post)
    ctx.expect_gap(
        turntable,
        body,
        axis="z",
        positive_elem=glass_disk,
        negative_elem=bottom_shell,
        min_gap=0.010,
        max_gap=0.016,
    )
    with ctx.pose({door_hinge: 1.35}):
        ctx.expect_gap(
            body,
            door,
            axis="x",
            positive_elem=control_frame,
            negative_elem=outer_frame,
            min_gap=0.44,
        )
    with ctx.pose({turntable_joint: math.pi / 2}):
        ctx.expect_contact(turntable, body, elem_a=glass_disk, elem_b=hub_post)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
