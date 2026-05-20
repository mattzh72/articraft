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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arc_points_2d(
    center_x: float,
    center_y: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append(
            (
                center_x + radius * math.cos(angle),
                center_y + radius * math.sin(angle),
            )
        )
    return points


def _build_palm_plate_mesh():
    front_arc = _arc_points_2d(
        0.030,
        0.0,
        0.024,
        math.radians(115.0),
        math.radians(-115.0),
        segments=9,
    )
    profile = [
        (-0.048, 0.026),
        (-0.041, 0.045),
        (-0.010, 0.055),
    ]
    profile.extend(front_arc)
    profile.extend(
        [
            (-0.010, -0.055),
            (-0.041, -0.045),
            (-0.048, -0.026),
        ]
    )
    return ExtrudeGeometry.from_z0(profile, 0.016, cap=True, closed=True)


def _add_knuckle_mount(
    palm_part,
    *,
    mount_name: str,
    joint_xyz: tuple[float, float, float],
    yaw: float,
    barrel_length: float,
    shell_material,
    hardware_material,
) -> None:
    ear_thickness = 0.0026
    ear_height = 0.010
    ear_depth = 0.010
    ear_offset = barrel_length * 0.5 + ear_thickness * 0.5

    palm_part.visual(
        Box((0.012, barrel_length + 0.006, 0.006)),
        origin=Origin(
            xyz=(
                joint_xyz[0] - 0.0065,
                joint_xyz[1],
                joint_xyz[2] - 0.008,
            ),
            rpy=(0.0, 0.0, yaw),
        ),
        material=shell_material,
        name=f"{mount_name}_pedestal",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        palm_part.visual(
            Box((ear_depth, ear_thickness, ear_height)),
            origin=Origin(
                xyz=(
                    joint_xyz[0] - 0.0045,
                    joint_xyz[1] + side_sign * ear_offset,
                    joint_xyz[2] - 0.001,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=hardware_material,
            name=f"{mount_name}_{side_name}_ear",
        )
    palm_part.visual(
        Box((0.008, barrel_length + 0.004, 0.004)),
        origin=Origin(
            xyz=(
                joint_xyz[0] - 0.0105,
                joint_xyz[1],
                joint_xyz[2] - 0.0035,
            ),
            rpy=(0.0, 0.0, yaw),
        ),
        material=hardware_material,
        name=f"{mount_name}_rear_web",
    )


def _build_finger_segment(
    part,
    *,
    prefix: str,
    segment_length: float,
    width: float,
    height: float,
    barrel_radius: float,
    barrel_length: float,
    shell_material,
    pad_material,
    hardware_material,
    include_distal_ears: bool,
    distal_barrel_length: float | None = None,
) -> None:
    body_clear = 0.010 if include_distal_ears else 0.003
    body_length = segment_length - barrel_radius - body_clear
    body_z = 0.0032

    part.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name=f"{prefix}_barrel",
    )
    part.visual(
        Box((body_length, width * 0.94, height * 0.70)),
        origin=Origin(
            xyz=(barrel_radius + body_length * 0.5, 0.0, body_z - height * 0.002),
        ),
        material=shell_material,
        name=f"{prefix}_lower_body",
    )
    part.visual(
        Box((body_length * 0.78, width * 0.72, height * 0.36)),
        origin=Origin(
            xyz=(barrel_radius + body_length * 0.48, 0.0, body_z + height * 0.32),
        ),
        material=shell_material,
        name=f"{prefix}_upper_body",
    )
    part.visual(
        Cylinder(radius=height * 0.34, length=width * 0.70),
        origin=Origin(
            xyz=(barrel_radius + body_length - height * 0.18, 0.0, body_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shell_material,
        name=f"{prefix}_tip_round",
    )
    part.visual(
        Box((body_length * 0.48, width * 0.64, height * 0.16)),
        origin=Origin(
            xyz=(barrel_radius + body_length * 0.58, 0.0, body_z - height * 0.33),
        ),
        material=pad_material,
        name=f"{prefix}_pad",
    )

    if include_distal_ears:
        ear_thickness = 0.0024
        ear_height = height * 0.92
        ear_depth = 0.008
        ear_gap_length = distal_barrel_length if distal_barrel_length is not None else barrel_length
        ear_offset = ear_gap_length * 0.5 + ear_thickness * 0.5
        connector_length = max(body_clear - 0.003, 0.0045)
        body_front_x = barrel_radius + body_length
        for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
            side_y = side_sign * ear_offset
            part.visual(
                Box((connector_length, ear_thickness, ear_height * 0.44)),
                origin=Origin(
                    xyz=(
                        body_front_x - 0.0005 + connector_length * 0.5,
                        side_y,
                        height * 0.06,
                    ),
                ),
                material=hardware_material,
                name=f"{prefix}_{side_name}_web",
            )
            part.visual(
                Box((ear_depth, ear_thickness, ear_height)),
                origin=Origin(
                    xyz=(segment_length - ear_depth * 0.5, side_y, 0.0),
                ),
                material=hardware_material,
                name=f"{prefix}_{side_name}_ear",
            )


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_manipulation_palm")

    palm_shell = model.material("palm_shell", rgba=(0.20, 0.23, 0.27, 1.0))
    finger_shell = model.material("finger_shell", rgba=(0.74, 0.77, 0.80, 1.0))
    finger_pad = model.material("finger_pad", rgba=(0.10, 0.11, 0.12, 1.0))
    hardware = model.material("hardware", rgba=(0.38, 0.41, 0.45, 1.0))
    heel_pad = model.material("heel_pad", rgba=(0.13, 0.14, 0.16, 1.0))

    palm = model.part("palm")
    palm.visual(
        mesh_from_geometry(_build_palm_plate_mesh(), "manipulation_palm_plate"),
        material=palm_shell,
        name="palm_plate",
    )
    palm.visual(
        Box((0.052, 0.070, 0.012)),
        origin=Origin(xyz=(-0.022, 0.0, 0.022)),
        material=palm_shell,
        name="palm_riser",
    )
    palm.visual(
        Box((0.044, 0.046, 0.018)),
        origin=Origin(xyz=(-0.040, 0.0, 0.016)),
        material=heel_pad,
        name="wrist_heel",
    )
    palm.visual(
        Box((0.024, 0.090, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.019)),
        material=hardware,
        name="knuckle_rail",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.118, 0.112, 0.040)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    finger_specs = [
        {
            "name": "left_outer",
            "base_xyz": (0.0365, 0.0440, 0.0220),
            "yaw": math.radians(17.0),
            "lengths": (0.036, 0.024, 0.018),
            "width": 0.0148,
            "prox_height": 0.0160,
        },
        {
            "name": "left_inner",
            "base_xyz": (0.0405, 0.0220, 0.0220),
            "yaw": 0.0,
            "lengths": (0.041, 0.028, 0.020),
            "width": 0.0158,
            "prox_height": 0.0170,
        },
        {
            "name": "center",
            "base_xyz": (0.0430, 0.0000, 0.0220),
            "yaw": 0.0,
            "lengths": (0.044, 0.030, 0.022),
            "width": 0.0166,
            "prox_height": 0.0178,
        },
        {
            "name": "right_inner",
            "base_xyz": (0.0405, -0.0220, 0.0220),
            "yaw": 0.0,
            "lengths": (0.041, 0.028, 0.020),
            "width": 0.0158,
            "prox_height": 0.0170,
        },
        {
            "name": "right_outer",
            "base_xyz": (0.0365, -0.0440, 0.0220),
            "yaw": math.radians(-17.0),
            "lengths": (0.036, 0.024, 0.018),
            "width": 0.0148,
            "prox_height": 0.0160,
        },
    ]

    for spec in finger_specs:
        name = spec["name"]
        base_xyz = spec["base_xyz"]
        yaw = spec["yaw"]
        proximal_length, middle_length, distal_length = spec["lengths"]
        width = spec["width"]
        proximal_height = spec["prox_height"]
        middle_height = proximal_height * 0.90
        distal_height = proximal_height * 0.80
        base_barrel_radius = width * 0.29
        hinge_barrel_length = width * 0.72

        _add_knuckle_mount(
            palm,
            mount_name=f"{name}_mount",
            joint_xyz=base_xyz,
            yaw=yaw,
            barrel_length=hinge_barrel_length,
            shell_material=palm_shell,
            hardware_material=hardware,
        )

        proximal = model.part(f"{name}_proximal")
        _build_finger_segment(
            proximal,
            prefix=f"{name}_proximal",
            segment_length=proximal_length,
            width=width,
            height=proximal_height,
            barrel_radius=base_barrel_radius,
            barrel_length=hinge_barrel_length,
            shell_material=finger_shell,
            pad_material=finger_pad,
            hardware_material=hardware,
            include_distal_ears=True,
            distal_barrel_length=hinge_barrel_length * 0.92,
        )
        proximal.inertial = Inertial.from_geometry(
            Box((proximal_length, width, proximal_height)),
            mass=0.085,
            origin=Origin(xyz=(proximal_length * 0.5, 0.0, 0.004)),
        )

        middle = model.part(f"{name}_middle")
        _build_finger_segment(
            middle,
            prefix=f"{name}_middle",
            segment_length=middle_length,
            width=width * 0.92,
            height=middle_height,
            barrel_radius=base_barrel_radius * 0.94,
            barrel_length=hinge_barrel_length * 0.92,
            shell_material=finger_shell,
            pad_material=finger_pad,
            hardware_material=hardware,
            include_distal_ears=True,
            distal_barrel_length=hinge_barrel_length * 0.84,
        )
        middle.inertial = Inertial.from_geometry(
            Box((middle_length, width * 0.92, middle_height)),
            mass=0.055,
            origin=Origin(xyz=(middle_length * 0.5, 0.0, 0.004)),
        )

        distal = model.part(f"{name}_distal")
        _build_finger_segment(
            distal,
            prefix=f"{name}_distal",
            segment_length=distal_length,
            width=width * 0.86,
            height=distal_height,
            barrel_radius=base_barrel_radius * 0.88,
            barrel_length=hinge_barrel_length * 0.84,
            shell_material=finger_shell,
            pad_material=finger_pad,
            hardware_material=hardware,
            include_distal_ears=False,
            distal_barrel_length=None,
        )
        distal.inertial = Inertial.from_geometry(
            Box((distal_length, width * 0.86, distal_height)),
            mass=0.035,
            origin=Origin(xyz=(distal_length * 0.5, 0.0, 0.004)),
        )

        palm_to_base = model.articulation(
            f"palm_to_{name}_base",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=base_xyz, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.5,
                lower=0.0,
                upper=1.10,
            ),
        )

        proximal_to_middle = model.articulation(
            f"{name}_proximal_to_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(proximal_length, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=3.0,
                lower=0.0,
                upper=1.25,
            ),
        )

        model.articulation(
            f"{name}_middle_to_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(middle_length, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=3.5,
                lower=0.0,
                upper=1.15,
            ),
        )

        palm_to_base.meta["finger_name"] = name
        proximal_to_middle.meta["finger_name"] = name

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
    palm = object_model.get_part("palm")
    finger_names = [
        "left_outer",
        "left_inner",
        "center",
        "right_inner",
        "right_outer",
    ]
    base_joints = [
        object_model.get_articulation(f"palm_to_{finger_name}_base")
        for finger_name in finger_names
    ]

    ctx.check(
        "five independent palm-edge base joints exist",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in base_joints)
        and len({joint.child for joint in base_joints}) == 5
        and all(joint.parent == palm.name for joint in base_joints),
        details=str(
            [
                (joint.name, joint.parent, joint.child, joint.articulation_type)
                for joint in base_joints
            ]
        ),
    )

    left_outer_base = object_model.get_articulation("palm_to_left_outer_base")
    right_outer_base = object_model.get_articulation("palm_to_right_outer_base")
    ctx.check(
        "outer finger bases are slightly splayed",
        left_outer_base.origin.rpy[2] > math.radians(10.0)
        and right_outer_base.origin.rpy[2] < math.radians(-10.0),
        details=(
            f"left_outer_yaw={left_outer_base.origin.rpy[2]:.3f}, "
            f"right_outer_yaw={right_outer_base.origin.rpy[2]:.3f}"
        ),
    )

    left_outer_origin = left_outer_base.origin.xyz
    left_inner_origin = object_model.get_articulation("palm_to_left_inner_base").origin.xyz
    center_origin = object_model.get_articulation("palm_to_center_base").origin.xyz
    right_inner_origin = object_model.get_articulation("palm_to_right_inner_base").origin.xyz
    right_outer_origin = right_outer_base.origin.xyz
    ctx.check(
        "finger bases follow a shallow front-edge arc",
        center_origin[0] > left_inner_origin[0] > left_outer_origin[0]
        and center_origin[0] > right_inner_origin[0] > right_outer_origin[0]
        and left_outer_origin[1] > left_inner_origin[1] > center_origin[1]
        and center_origin[1] > right_inner_origin[1] > right_outer_origin[1],
        details=(
            f"left_outer={left_outer_origin}, left_inner={left_inner_origin}, "
            f"center={center_origin}, right_inner={right_inner_origin}, "
            f"right_outer={right_outer_origin}"
        ),
    )

    center_distal = object_model.get_part("center_distal")
    center_base = object_model.get_articulation("palm_to_center_base")
    center_mid = object_model.get_articulation("center_proximal_to_middle")
    center_tip = object_model.get_articulation("center_middle_to_distal")
    center_rest = ctx.part_world_position(center_distal)
    with ctx.pose({center_base: 0.34, center_mid: 0.46, center_tip: 0.30}):
        center_flexed = ctx.part_world_position(center_distal)
    ctx.check(
        "center finger curls upward about its chain",
        center_rest is not None
        and center_flexed is not None
        and center_flexed[2] > center_rest[2] + 0.016
        and center_flexed[0] < center_rest[0] - 0.010,
        details=f"rest={center_rest}, flexed={center_flexed}",
    )

    left_outer_distal = object_model.get_part("left_outer_distal")
    center_distal_rest = ctx.part_world_position(center_distal)
    left_outer_rest = ctx.part_world_position(left_outer_distal)
    with ctx.pose({left_outer_base: 0.42}):
        center_distal_after = ctx.part_world_position(center_distal)
        left_outer_after = ctx.part_world_position(left_outer_distal)
    unchanged = (
        center_distal_rest is not None
        and center_distal_after is not None
        and _distance(center_distal_rest, center_distal_after) < 1e-6
    )
    moved = (
        left_outer_rest is not None
        and left_outer_after is not None
        and _distance(left_outer_rest, left_outer_after) > 0.012
    )
    ctx.check(
        "moving one base joint leaves neighboring chains untouched",
        unchanged and moved,
        details=(
            f"center_rest={center_distal_rest}, center_after={center_distal_after}, "
            f"left_outer_rest={left_outer_rest}, left_outer_after={left_outer_after}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
