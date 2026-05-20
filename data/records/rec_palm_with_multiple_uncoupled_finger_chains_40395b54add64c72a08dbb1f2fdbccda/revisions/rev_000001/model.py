from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    corner: float,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z + z_center) for y, z in rounded_rect_profile(width, height, corner)]


def _build_palm_shell_mesh():
    floor = section_loft(
        [
            _yz_section(-0.045, width=0.070, height=0.008, corner=0.0030, z_center=0.0030),
            _yz_section(-0.014, width=0.086, height=0.0085, corner=0.0034, z_center=0.0024),
            _yz_section(0.022, width=0.094, height=0.0095, corner=0.0038, z_center=0.0032),
            _yz_section(0.056, width=0.078, height=0.0110, corner=0.0032, z_center=0.0055),
        ]
    )

    rear_wall = BoxGeometry((0.018, 0.074, 0.020)).translate(-0.047, 0.0, 0.013)
    left_wall = BoxGeometry((0.082, 0.010, 0.022)).translate(0.002, 0.046, 0.015)
    right_wall = BoxGeometry((0.082, 0.010, 0.022)).translate(0.002, -0.046, 0.015)
    front_lip = BoxGeometry((0.020, 0.072, 0.018)).translate(0.059, 0.0, 0.016)

    floor.merge(rear_wall)
    floor.merge(left_wall)
    floor.merge(right_wall)
    floor.merge(front_lip)
    return floor


def _build_link_shell_mesh(
    *,
    name: str,
    length: float,
    width: float,
    thickness: float,
    root_fraction: float = 0.08,
) -> object:
    root_width = width * 0.98
    mid_width = width * 0.88
    tip_width = width * 0.72
    root_height = thickness * 0.98
    mid_height = thickness * 0.88
    tip_height = thickness * 0.68

    shell = section_loft(
        [
            _yz_section(length * root_fraction, width=root_width, height=root_height, corner=min(width, thickness) * 0.22),
            _yz_section(length * 0.52, width=mid_width, height=mid_height, corner=min(width, thickness) * 0.20),
            _yz_section(length, width=tip_width, height=tip_height, corner=min(width, thickness) * 0.18, z_center=-thickness * 0.03),
        ]
    )
    return mesh_from_geometry(shell, name)


def _add_link(
    part,
    *,
    shell_mesh,
    body_material,
    joint_material,
    pad_material,
    length: float,
    width: float,
    thickness: float,
    mass: float,
) -> None:
    knuckle_radius = thickness * 0.34

    part.visual(
        Cylinder(radius=knuckle_radius, length=width * 0.94),
        origin=Origin(xyz=(knuckle_radius, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_material,
        name="root_knuckle",
    )
    part.visual(shell_mesh, material=body_material, name="link_shell")
    part.inertial = Inertial.from_geometry(
        Box((length, width, thickness)),
        mass=mass,
        origin=Origin(xyz=(length * 0.50, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_cupped_palm")

    shell_graphite = model.material("shell_graphite", rgba=(0.23, 0.25, 0.29, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    joint_steel = model.material("joint_steel", rgba=(0.67, 0.70, 0.74, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.09, 0.10, 0.11, 1.0))

    front_outer_prox_mesh = _build_link_shell_mesh(
        name="front_outer_proximal_shell",
        length=0.042,
        width=0.017,
        thickness=0.017,
    )
    front_outer_mid_mesh = _build_link_shell_mesh(
        name="front_outer_middle_shell",
        length=0.028,
        width=0.015,
        thickness=0.014,
    )
    front_outer_dist_mesh = _build_link_shell_mesh(
        name="front_outer_distal_shell",
        length=0.021,
        width=0.013,
        thickness=0.012,
    )
    front_center_prox_mesh = _build_link_shell_mesh(
        name="front_center_proximal_shell",
        length=0.047,
        width=0.0185,
        thickness=0.018,
    )
    front_center_mid_mesh = _build_link_shell_mesh(
        name="front_center_middle_shell",
        length=0.032,
        width=0.0165,
        thickness=0.015,
    )
    front_center_dist_mesh = _build_link_shell_mesh(
        name="front_center_distal_shell",
        length=0.024,
        width=0.0145,
        thickness=0.013,
    )
    side_prox_mesh = _build_link_shell_mesh(
        name="side_proximal_shell",
        length=0.034,
        width=0.016,
        thickness=0.016,
        root_fraction=0.30,
    )
    side_mid_mesh = _build_link_shell_mesh(
        name="side_middle_shell",
        length=0.026,
        width=0.014,
        thickness=0.014,
        root_fraction=0.30,
    )
    side_dist_mesh = _build_link_shell_mesh(
        name="side_distal_shell",
        length=0.019,
        width=0.012,
        thickness=0.012,
        root_fraction=0.30,
    )

    palm = model.part("palm")
    palm.visual(
        mesh_from_geometry(_build_palm_shell_mesh(), "palm_shell"),
        material=shell_graphite,
        name="palm_shell",
    )
    palm.visual(
        Box((0.016, 0.018, 0.012)),
        origin=Origin(xyz=(0.064, 0.028, 0.027)),
        material=shell_dark,
        name="front_mount_left",
    )
    palm.visual(
        Box((0.016, 0.018, 0.012)),
        origin=Origin(xyz=(0.064, 0.000, 0.027)),
        material=shell_dark,
        name="front_mount_center",
    )
    palm.visual(
        Box((0.016, 0.018, 0.012)),
        origin=Origin(xyz=(0.064, -0.028, 0.027)),
        material=shell_dark,
        name="front_mount_right",
    )
    palm.visual(
        Box((0.004, 0.018, 0.024)),
        origin=Origin(xyz=(0.016, 0.063, 0.026)),
        material=shell_dark,
        name="side_mount_left",
    )
    palm.visual(
        Box((0.020, 0.016, 0.010)),
        origin=Origin(xyz=(0.014, 0.056, 0.010)),
        material=shell_dark,
        name="side_mount_left_brace",
    )
    palm.visual(
        Box((0.004, 0.018, 0.024)),
        origin=Origin(xyz=(0.000, -0.063, 0.026)),
        material=shell_dark,
        name="side_mount_right",
    )
    palm.visual(
        Box((0.020, 0.016, 0.010)),
        origin=Origin(xyz=(0.006, -0.056, 0.010)),
        material=shell_dark,
        name="side_mount_right_brace",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.118, 0.110, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.004, 0.0, 0.015)),
    )

    chain_specs = [
        {
            "name": "front_left",
            "base_origin": Origin(xyz=(0.072, 0.028, 0.027)),
            "base_axis": (0.0, 1.0, 0.0),
            "mount_visual": "front_mount_left",
            "links": [
                (front_outer_prox_mesh, 0.042, 0.017, 0.017, 0.060),
                (front_outer_mid_mesh, 0.028, 0.015, 0.014, 0.036),
                (front_outer_dist_mesh, 0.021, 0.013, 0.012, 0.024),
            ],
        },
        {
            "name": "front_center",
            "base_origin": Origin(xyz=(0.072, 0.000, 0.027)),
            "base_axis": (0.0, 1.0, 0.0),
            "mount_visual": "front_mount_center",
            "links": [
                (front_center_prox_mesh, 0.047, 0.0185, 0.018, 0.066),
                (front_center_mid_mesh, 0.032, 0.0165, 0.015, 0.040),
                (front_center_dist_mesh, 0.024, 0.0145, 0.013, 0.026),
            ],
        },
        {
            "name": "front_right",
            "base_origin": Origin(xyz=(0.072, -0.028, 0.027)),
            "base_axis": (0.0, 1.0, 0.0),
            "mount_visual": "front_mount_right",
            "links": [
                (front_outer_prox_mesh, 0.042, 0.017, 0.017, 0.060),
                (front_outer_mid_mesh, 0.028, 0.015, 0.014, 0.036),
                (front_outer_dist_mesh, 0.021, 0.013, 0.012, 0.024),
            ],
        },
        {
            "name": "side_left",
            "base_origin": Origin(xyz=(0.018, 0.063, 0.026)),
            "base_axis": (0.0, 1.0, 0.0),
            "mount_visual": "side_mount_left",
            "links": [
                (side_prox_mesh, 0.034, 0.016, 0.016, 0.048),
                (side_mid_mesh, 0.026, 0.014, 0.014, 0.030),
                (side_dist_mesh, 0.019, 0.012, 0.012, 0.018),
            ],
        },
        {
            "name": "side_right",
            "base_origin": Origin(xyz=(0.002, -0.063, 0.026)),
            "base_axis": (0.0, 1.0, 0.0),
            "mount_visual": "side_mount_right",
            "links": [
                (side_prox_mesh, 0.034, 0.016, 0.016, 0.048),
                (side_mid_mesh, 0.026, 0.014, 0.014, 0.030),
                (side_dist_mesh, 0.019, 0.012, 0.012, 0.018),
            ],
        },
    ]

    for spec in chain_specs:
        proximal = model.part(f"{spec['name']}_proximal")
        middle = model.part(f"{spec['name']}_middle")
        distal = model.part(f"{spec['name']}_distal")

        prox_mesh, prox_len, prox_w, prox_t, prox_mass = spec["links"][0]
        mid_mesh, mid_len, mid_w, mid_t, mid_mass = spec["links"][1]
        dist_mesh, dist_len, dist_w, dist_t, dist_mass = spec["links"][2]

        _add_link(
            proximal,
            shell_mesh=prox_mesh,
            body_material=shell_graphite,
            joint_material=joint_steel,
            pad_material=pad_rubber,
            length=prox_len,
            width=prox_w,
            thickness=prox_t,
            mass=prox_mass,
        )
        _add_link(
            middle,
            shell_mesh=mid_mesh,
            body_material=shell_graphite,
            joint_material=joint_steel,
            pad_material=pad_rubber,
            length=mid_len,
            width=mid_w,
            thickness=mid_t,
            mass=mid_mass,
        )
        _add_link(
            distal,
            shell_mesh=dist_mesh,
            body_material=shell_graphite,
            joint_material=joint_steel,
            pad_material=pad_rubber,
            length=dist_len,
            width=dist_w,
            thickness=dist_t,
            mass=dist_mass,
        )

        model.articulation(
            f"palm_to_{spec['name']}_proximal",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=spec["base_origin"],
            axis=spec["base_axis"],
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.0,
                lower=0.0,
                upper=1.25 if spec["name"].startswith("side_") else 1.35,
            ),
        )
        model.articulation(
            f"{spec['name']}_proximal_to_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(prox_len, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.2, velocity=2.4, lower=0.0, upper=1.35),
        )
        model.articulation(
            f"{spec['name']}_middle_to_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(mid_len, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.6, velocity=2.6, lower=0.0, upper=1.10),
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

    palm = object_model.get_part("palm")
    front_left_proximal = object_model.get_part("front_left_proximal")
    front_center_proximal = object_model.get_part("front_center_proximal")
    front_right_proximal = object_model.get_part("front_right_proximal")
    side_left_proximal = object_model.get_part("side_left_proximal")
    side_right_proximal = object_model.get_part("side_right_proximal")
    front_center_distal = object_model.get_part("front_center_distal")
    side_left_distal = object_model.get_part("side_left_distal")

    center_base = object_model.get_articulation("palm_to_front_center_proximal")
    center_middle = object_model.get_articulation("front_center_proximal_to_middle")
    center_distal = object_model.get_articulation("front_center_middle_to_distal")
    side_left_base = object_model.get_articulation("palm_to_side_left_proximal")
    side_left_middle = object_model.get_articulation("side_left_proximal_to_middle")
    side_left_distal_joint = object_model.get_articulation("side_left_middle_to_distal")

    ctx.expect_contact(
        front_left_proximal,
        palm,
        elem_a="root_knuckle",
        elem_b="front_mount_left",
        name="left front chain base knuckle seats on left front mount",
    )
    ctx.expect_contact(
        front_center_proximal,
        palm,
        elem_a="root_knuckle",
        elem_b="front_mount_center",
        name="center front chain base knuckle seats on center front mount",
    )
    ctx.expect_contact(
        front_right_proximal,
        palm,
        elem_a="root_knuckle",
        elem_b="front_mount_right",
        name="right front chain base knuckle seats on right front mount",
    )
    ctx.expect_contact(
        side_left_proximal,
        palm,
        elem_a="root_knuckle",
        elem_b="side_mount_left",
        name="left side chain base knuckle seats on left side wall mount",
    )
    ctx.expect_contact(
        side_right_proximal,
        palm,
        elem_a="root_knuckle",
        elem_b="side_mount_right",
        name="right side chain base knuckle seats on right side wall mount",
    )

    front_left_pos = ctx.part_world_position(front_left_proximal)
    front_center_pos = ctx.part_world_position(front_center_proximal)
    front_right_pos = ctx.part_world_position(front_right_proximal)
    side_left_pos = ctx.part_world_position(side_left_proximal)
    side_right_pos = ctx.part_world_position(side_right_proximal)

    ctx.check(
        "front chains span the palm front edge",
        front_left_pos is not None
        and front_center_pos is not None
        and front_right_pos is not None
        and front_left_pos[0] > 0.065
        and front_center_pos[0] > 0.065
        and front_right_pos[0] > 0.065
        and front_left_pos[1] > front_center_pos[1] > front_right_pos[1]
        and (front_left_pos[1] - front_center_pos[1]) > 0.020
        and (front_center_pos[1] - front_right_pos[1]) > 0.020,
        details=f"left={front_left_pos}, center={front_center_pos}, right={front_right_pos}",
    )
    ctx.check(
        "side chains mount on opposite palm walls",
        side_left_pos is not None
        and side_right_pos is not None
        and side_left_pos[1] > 0.060
        and side_right_pos[1] < -0.060,
        details=f"left={side_left_pos}, right={side_right_pos}",
    )

    left_front_rest = ctx.part_world_position(front_left_proximal)
    center_distal_rest = ctx.part_world_position(front_center_distal)
    with ctx.pose({center_base: 0.72, center_middle: 0.95, center_distal: 0.80}):
        left_front_after = ctx.part_world_position(front_left_proximal)
        center_distal_curled = ctx.part_world_position(front_center_distal)

    left_thumb_rest = ctx.part_world_position(side_left_distal)
    with ctx.pose({side_left_base: 0.65, side_left_middle: 0.78, side_left_distal_joint: 0.70}):
        left_thumb_curled = ctx.part_world_position(side_left_distal)

    front_independent = (
        left_front_rest is not None
        and left_front_after is not None
        and center_distal_rest is not None
        and center_distal_curled is not None
        and abs(left_front_rest[0] - left_front_after[0]) < 1e-6
        and abs(left_front_rest[1] - left_front_after[1]) < 1e-6
        and abs(left_front_rest[2] - left_front_after[2]) < 1e-6
        and center_distal_curled[2] < center_distal_rest[2] - 0.020
        and center_distal_curled[0] < center_distal_rest[0] - 0.015
    )
    ctx.check(
        "center front chain curls while neighboring base remains independent",
        front_independent,
        details=(
            f"left_rest={left_front_rest}, left_after={left_front_after}, "
            f"center_rest={center_distal_rest}, center_curled={center_distal_curled}"
        ),
    )

    ctx.check(
        "left side chain curls downward from its side-wall base",
        left_thumb_rest is not None
        and left_thumb_curled is not None
        and left_thumb_curled[2] < left_thumb_rest[2] - 0.016,
        details=f"rest={left_thumb_rest}, curled={left_thumb_curled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
