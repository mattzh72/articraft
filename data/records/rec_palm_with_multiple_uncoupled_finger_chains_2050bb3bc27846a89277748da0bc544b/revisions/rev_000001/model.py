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


def _build_finger_segment(
    part,
    *,
    length: float,
    width: float,
    thickness: float,
    shell_material,
    pad_material,
    joint_material,
    mass: float,
) -> float:
    barrel_radius = min(width * 0.34, thickness * 0.42)

    part.visual(
        Cylinder(radius=barrel_radius, length=width * 0.82),
        origin=Origin(xyz=(0.0, 0.0, barrel_radius), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((width * 0.58, thickness * 0.28, barrel_radius * 0.90)),
        origin=Origin(
            xyz=(
                0.0,
                -thickness * 0.04,
                barrel_radius * 0.45,
            )
        ),
        material=joint_material,
        name="root_shoe",
    )
    part.visual(
        Box((width * 0.92, thickness * 0.90, length)),
        origin=Origin(xyz=(0.0, 0.0, barrel_radius + length * 0.5)),
        material=shell_material,
        name="segment_body",
    )
    part.visual(
        Box((width * 0.70, thickness * 0.36, length * 0.70)),
        origin=Origin(
            xyz=(
                0.0,
                -thickness * 0.20,
                barrel_radius + length * 0.52,
            )
        ),
        material=pad_material,
        name="segment_pad",
    )
    part.visual(
        Box((width * 0.70, thickness * 0.72, max(length * 0.22, 0.008))),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                barrel_radius + length - max(length * 0.22, 0.008) * 0.5,
            )
        ),
        material=shell_material,
        name="tip_block",
    )
    part.inertial = Inertial.from_geometry(
        Box((width, thickness, length + barrel_radius)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, barrel_radius + length * 0.5)),
    )
    return barrel_radius + length


def _build_thumb_segment(
    part,
    *,
    length: float,
    width: float,
    thickness: float,
    shell_material,
    pad_material,
    joint_material,
    mass: float,
) -> float:
    barrel_radius = min(width * 0.34, thickness * 0.42)

    part.visual(
        Cylinder(radius=barrel_radius, length=width * 0.84),
        origin=Origin(xyz=(barrel_radius, 0.0, 0.0)),
        material=joint_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((barrel_radius * 0.90, thickness * 0.28, width * 0.58)),
        origin=Origin(
            xyz=(
                barrel_radius * 0.45,
                -thickness * 0.04,
                0.0,
            )
        ),
        material=joint_material,
        name="root_shoe",
    )
    part.visual(
        Box((length, thickness * 0.90, width * 0.90)),
        origin=Origin(xyz=(barrel_radius + length * 0.5, 0.0, 0.0)),
        material=shell_material,
        name="segment_body",
    )
    part.visual(
        Box((length * 0.66, thickness * 0.36, width * 0.68)),
        origin=Origin(
            xyz=(
                barrel_radius + length * 0.54,
                -thickness * 0.20,
                0.0,
            )
        ),
        material=pad_material,
        name="segment_pad",
    )
    part.visual(
        Box((max(length * 0.24, 0.008), thickness * 0.74, width * 0.72)),
        origin=Origin(
            xyz=(
                barrel_radius + length - max(length * 0.24, 0.008) * 0.5,
                0.0,
                0.0,
            )
        ),
        material=shell_material,
        name="tip_block",
    )
    part.inertial = Inertial.from_geometry(
        Box((length + barrel_radius, thickness, width)),
        mass=mass,
        origin=Origin(xyz=(barrel_radius + length * 0.5, 0.0, 0.0)),
    )
    return barrel_radius + length


def _add_finger_chain(
    model: ArticulatedObject,
    palm,
    *,
    name: str,
    root_xyz: tuple[float, float, float],
    lengths: tuple[float, float, float],
    width: float,
    thickness: float,
    shell_material,
    pad_material,
    joint_material,
    root_upper: float = 1.35,
) -> None:
    proximal = model.part(f"{name}_proximal")
    middle = model.part(f"{name}_middle")
    distal = model.part(f"{name}_distal")

    proximal_end = _build_finger_segment(
        proximal,
        length=lengths[0],
        width=width,
        thickness=thickness,
        shell_material=shell_material,
        pad_material=pad_material,
        joint_material=joint_material,
        mass=0.040,
    )
    middle_end = _build_finger_segment(
        middle,
        length=lengths[1],
        width=width * 0.92,
        thickness=thickness * 0.92,
        shell_material=shell_material,
        pad_material=pad_material,
        joint_material=joint_material,
        mass=0.030,
    )
    distal_end = _build_finger_segment(
        distal,
        length=lengths[2],
        width=width * 0.84,
        thickness=thickness * 0.84,
        shell_material=shell_material,
        pad_material=pad_material,
        joint_material=joint_material,
        mass=0.022,
    )

    model.articulation(
        f"palm_to_{name}_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=proximal,
        origin=Origin(xyz=root_xyz),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=4.0,
            lower=-0.12,
            upper=root_upper,
        ),
    )
    model.articulation(
        f"{name}_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, proximal_end)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=4.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        f"{name}_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.0, 0.0, middle_end)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    distal.meta["segment_end"] = distal_end


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modular_robotic_palm")

    shell_silver = model.material("shell_silver", rgba=(0.72, 0.75, 0.79, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    elastomer = model.material("elastomer", rgba=(0.12, 0.13, 0.15, 1.0))
    joint_black = model.material("joint_black", rgba=(0.07, 0.08, 0.09, 1.0))
    finger_metal = model.material("finger_metal", rgba=(0.58, 0.61, 0.66, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    palm = model.part("palm_housing")
    palm.visual(
        Box((0.104, 0.004, 0.124)),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=shell_silver,
        name="front_face_plate",
    )
    palm.visual(
        Box((0.006, 0.024, 0.118)),
        origin=Origin(xyz=(-0.049, 0.002, 0.0)),
        material=shell_silver,
        name="left_side_wall",
    )
    palm.visual(
        Box((0.006, 0.024, 0.118)),
        origin=Origin(xyz=(0.049, 0.002, 0.0)),
        material=shell_silver,
        name="right_side_wall",
    )
    palm.visual(
        Box((0.104, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.058)),
        material=shell_silver,
        name="top_crossbeam",
    )
    palm.visual(
        Box((0.078, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.002, -0.053)),
        material=shell_silver,
        name="wrist_crossbeam",
    )
    palm.visual(
        Box((0.038, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.072)),
        material=shell_dark,
        name="wrist_socket",
    )
    palm.visual(
        Box((0.074, 0.002, 0.074)),
        origin=Origin(xyz=(0.0, -0.015, -0.004)),
        material=elastomer,
        name="palm_pad",
    )
    palm.visual(
        Box((0.088, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.012, 0.055)),
        material=shell_dark,
        name="rear_top_frame",
    )
    palm.visual(
        Box((0.062, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.012, -0.050)),
        material=shell_dark,
        name="rear_bottom_frame",
    )
    palm.visual(
        Box((0.006, 0.004, 0.094)),
        origin=Origin(xyz=(-0.041, 0.012, 0.001)),
        material=shell_dark,
        name="rear_left_frame",
    )
    palm.visual(
        Box((0.006, 0.004, 0.094)),
        origin=Origin(xyz=(0.041, 0.012, 0.001)),
        material=shell_dark,
        name="rear_right_frame",
    )

    finger_mount_x = {
        "index": -0.033,
        "middle": -0.011,
        "ring": 0.011,
        "pinky": 0.032,
    }
    for finger_name, x_pos in finger_mount_x.items():
        palm.visual(
            Box((0.014, 0.008, 0.012)),
            origin=Origin(xyz=(x_pos, -0.014, 0.059)),
            material=joint_black,
            name=f"{finger_name}_mount",
        )

    palm.visual(
        Box((0.010, 0.010, 0.024)),
        origin=Origin(xyz=(0.056, -0.014, 0.020)),
        material=joint_black,
        name="thumb_mount",
    )
    palm.visual(
        Box((0.074, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.011, 0.061)),
        material=joint_black,
        name="panel_hinge_pad",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.104, 0.028, 0.150)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.076, 0.003, 0.086)),
        origin=Origin(xyz=(0.0, 0.0015, -0.043)),
        material=panel_dark,
        name="panel_leaf",
    )
    service_panel.visual(
        Box((0.060, 0.005, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, -0.023)),
        material=shell_dark,
        name="upper_reinforcement",
    )
    service_panel.visual(
        Box((0.060, 0.005, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, -0.062)),
        material=shell_dark,
        name="lower_reinforcement",
    )
    for index, x_pos in enumerate((-0.024, 0.0, 0.024)):
        service_panel.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(
                xyz=(x_pos, 0.004, -0.004),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=joint_black,
            name=f"hinge_barrel_{index}",
        )
    service_panel.inertial = Inertial.from_geometry(
        Box((0.076, 0.012, 0.086)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.004, -0.043)),
    )
    model.articulation(
        "palm_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=service_panel,
        origin=Origin(xyz=(0.0, 0.014, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    _add_finger_chain(
        model,
        palm,
        name="index",
        root_xyz=(finger_mount_x["index"], -0.018, 0.063),
        lengths=(0.046, 0.028, 0.022),
        width=0.018,
        thickness=0.018,
        shell_material=finger_metal,
        pad_material=elastomer,
        joint_material=joint_black,
    )
    _add_finger_chain(
        model,
        palm,
        name="middle",
        root_xyz=(finger_mount_x["middle"], -0.018, 0.063),
        lengths=(0.050, 0.031, 0.024),
        width=0.019,
        thickness=0.0185,
        shell_material=finger_metal,
        pad_material=elastomer,
        joint_material=joint_black,
    )
    _add_finger_chain(
        model,
        palm,
        name="ring",
        root_xyz=(finger_mount_x["ring"], -0.018, 0.063),
        lengths=(0.047, 0.029, 0.022),
        width=0.018,
        thickness=0.018,
        shell_material=finger_metal,
        pad_material=elastomer,
        joint_material=joint_black,
    )
    _add_finger_chain(
        model,
        palm,
        name="pinky",
        root_xyz=(finger_mount_x["pinky"], -0.018, 0.063),
        lengths=(0.039, 0.024, 0.018),
        width=0.015,
        thickness=0.0155,
        shell_material=finger_metal,
        pad_material=elastomer,
        joint_material=joint_black,
        root_upper=1.45,
    )

    thumb_metacarpal = model.part("thumb_metacarpal")
    thumb_proximal = model.part("thumb_proximal")
    thumb_distal = model.part("thumb_distal")

    thumb_metacarpal_end = _build_thumb_segment(
        thumb_metacarpal,
        length=0.028,
        width=0.018,
        thickness=0.017,
        shell_material=finger_metal,
        pad_material=elastomer,
        joint_material=joint_black,
        mass=0.032,
    )
    thumb_proximal_end = _build_thumb_segment(
        thumb_proximal,
        length=0.024,
        width=0.017,
        thickness=0.016,
        shell_material=finger_metal,
        pad_material=elastomer,
        joint_material=joint_black,
        mass=0.026,
    )
    thumb_distal_end = _build_thumb_segment(
        thumb_distal,
        length=0.019,
        width=0.015,
        thickness=0.014,
        shell_material=finger_metal,
        pad_material=elastomer,
        joint_material=joint_black,
        mass=0.018,
    )

    model.articulation(
        "palm_to_thumb_metacarpal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_metacarpal,
        origin=Origin(xyz=(0.061, -0.014, 0.020)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.10,
            upper=1.05,
        ),
    )
    model.articulation(
        "thumb_metacarpal_to_proximal",
        ArticulationType.REVOLUTE,
        parent=thumb_metacarpal,
        child=thumb_proximal,
        origin=Origin(xyz=(thumb_metacarpal_end, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "thumb_proximal_to_distal",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_distal,
        origin=Origin(xyz=(thumb_proximal_end, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    thumb_distal.meta["segment_end"] = thumb_distal_end

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
    palm = object_model.get_part("palm_housing")
    service_panel = object_model.get_part("service_panel")
    panel_hinge = object_model.get_articulation("palm_to_service_panel")
    thumb_base = object_model.get_articulation("palm_to_thumb_metacarpal")
    thumb_mid = object_model.get_articulation("thumb_metacarpal_to_proximal")
    thumb_tip = object_model.get_articulation("thumb_proximal_to_distal")
    thumb_distal = object_model.get_part("thumb_distal")

    ctx.expect_gap(
        service_panel,
        palm,
        axis="y",
        min_gap=0.0,
        max_gap=0.005,
        positive_elem="panel_leaf",
        name="service panel sits flush on the rear face",
    )
    ctx.expect_overlap(
        service_panel,
        palm,
        axes="xz",
        min_overlap=0.050,
        elem_a="panel_leaf",
        name="service panel covers the rear access opening",
    )

    palm_mount_joints = [
        object_model.get_articulation("palm_to_index_proximal"),
        object_model.get_articulation("palm_to_middle_proximal"),
        object_model.get_articulation("palm_to_ring_proximal"),
        object_model.get_articulation("palm_to_pinky_proximal"),
        thumb_base,
        panel_hinge,
    ]
    ctx.check(
        "palm carries four finger roots plus thumb and panel hinges",
        len(palm_mount_joints) == 6
        and all(joint.parent == "palm_housing" for joint in palm_mount_joints),
        details=str([(joint.name, joint.parent, joint.child) for joint in palm_mount_joints]),
    )

    finger_names = ("index", "middle", "ring", "pinky")
    finger_root_x = []
    for finger_name in finger_names:
        root_joint = object_model.get_articulation(f"palm_to_{finger_name}_proximal")
        mid_joint = object_model.get_articulation(f"{finger_name}_proximal_to_middle")
        tip_joint = object_model.get_articulation(f"{finger_name}_middle_to_distal")
        finger_root_x.append(root_joint.origin.xyz[0])

        ctx.check(
            f"{finger_name} finger uses three revolute links",
            root_joint.articulation_type == ArticulationType.REVOLUTE
            and mid_joint.articulation_type == ArticulationType.REVOLUTE
            and tip_joint.articulation_type == ArticulationType.REVOLUTE,
            details=str(
                (
                    root_joint.articulation_type,
                    mid_joint.articulation_type,
                    tip_joint.articulation_type,
                )
            ),
        )

    ctx.check(
        "finger root joints are independently spaced across the palm edge",
        finger_root_x == sorted(finger_root_x)
        and min(
            finger_root_x[index + 1] - finger_root_x[index]
            for index in range(len(finger_root_x) - 1)
        )
        > 0.018,
        details=str(finger_root_x),
    )
    ctx.check(
        "thumb joint is mounted on the side of the palm",
        thumb_base.origin.xyz[0] > 0.050 and thumb_base.origin.xyz[2] < 0.035,
        details=str(thumb_base.origin.xyz),
    )

    rest_finger_centers = {}
    for finger_name in finger_names:
        distal = object_model.get_part(f"{finger_name}_distal")
        rest_finger_centers[finger_name] = _center_from_aabb(ctx.part_world_aabb(distal))
    rest_thumb_center = _center_from_aabb(ctx.part_world_aabb(thumb_distal))
    closed_panel_box = ctx.part_element_world_aabb(service_panel, elem="panel_leaf")

    flex_pose = {
        object_model.get_articulation("palm_to_index_proximal"): 0.85,
        object_model.get_articulation("index_proximal_to_middle"): 1.00,
        object_model.get_articulation("index_middle_to_distal"): 0.82,
        object_model.get_articulation("palm_to_middle_proximal"): 0.92,
        object_model.get_articulation("middle_proximal_to_middle"): 1.05,
        object_model.get_articulation("middle_middle_to_distal"): 0.86,
        object_model.get_articulation("palm_to_ring_proximal"): 0.88,
        object_model.get_articulation("ring_proximal_to_middle"): 1.00,
        object_model.get_articulation("ring_middle_to_distal"): 0.82,
        object_model.get_articulation("palm_to_pinky_proximal"): 0.96,
        object_model.get_articulation("pinky_proximal_to_middle"): 1.02,
        object_model.get_articulation("pinky_middle_to_distal"): 0.80,
        thumb_base: 0.65,
        thumb_mid: 0.78,
        thumb_tip: 0.70,
        panel_hinge: 1.05,
    }
    with ctx.pose(flex_pose):
        for finger_name in finger_names:
            distal = object_model.get_part(f"{finger_name}_distal")
            flex_center = _center_from_aabb(ctx.part_world_aabb(distal))
            rest_center = rest_finger_centers[finger_name]
            ctx.check(
                f"{finger_name} fingertip curls toward the palm",
                rest_center is not None
                and flex_center is not None
                and flex_center[1] < rest_center[1] - 0.010
                and flex_center[2] < rest_center[2] - 0.020,
                details=f"rest={rest_center}, flex={flex_center}",
            )

        flex_thumb_center = _center_from_aabb(ctx.part_world_aabb(thumb_distal))
        ctx.check(
            "thumb swings inward from the side mount",
            rest_thumb_center is not None
            and flex_thumb_center is not None
            and flex_thumb_center[1] < rest_thumb_center[1] - 0.012
            and flex_thumb_center[0] < rest_thumb_center[0] - 0.010,
            details=f"rest={rest_thumb_center}, flex={flex_thumb_center}",
        )

        open_panel_box = ctx.part_element_world_aabb(service_panel, elem="panel_leaf")
        ctx.check(
            "service panel flips open away from the rear face",
            closed_panel_box is not None
            and open_panel_box is not None
            and open_panel_box[1][1] > closed_panel_box[1][1] + 0.020,
            details=f"closed={closed_panel_box}, open={open_panel_box}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
