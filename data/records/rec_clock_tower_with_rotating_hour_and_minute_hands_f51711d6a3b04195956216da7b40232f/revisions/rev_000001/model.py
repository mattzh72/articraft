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
    mesh_from_geometry,
    section_loft,
)


OUTER_WIDTH = 2.8
OUTER_DEPTH = 2.2
LOWER_HEIGHT = 6.4
UPPER_HEIGHT = 1.6
ROOF_HEIGHT = 1.5
TOP_OF_SHAFT = LOWER_HEIGHT + UPPER_HEIGHT

LOWER_WALL_WIDTH = 2.32
LOWER_WALL_DEPTH = 1.72
UPPER_WALL_WIDTH = 2.20
UPPER_WALL_DEPTH = 1.60

DIAL_SIZE = 1.00
DIAL_THICKNESS = 0.04
CLOCK_CENTER_Z = LOWER_HEIGHT + 0.70

FACES = ("north", "south", "east", "west")


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_axis_z_to_vector(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    thickness: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((thickness, thickness, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_axis_z_to_vector(a, b)),
        material=material,
        name=name,
    )


def _rect_loop(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d, z),
        (half_w, -half_d, z),
        (half_w, half_d, z),
        (-half_w, half_d, z),
    ]


def _face_spec(face: str) -> dict[str, object]:
    if face == "north":
        return {
            "axis": (0.0, 1.0, 0.0),
            "kind": "y",
            "sign": 1.0,
            "cyl_rpy": (-math.pi / 2.0, 0.0, 0.0),
            "origin": (0.0, UPPER_WALL_DEPTH * 0.5 + DIAL_THICKNESS, CLOCK_CENTER_Z),
            "dial_center": (0.0, UPPER_WALL_DEPTH * 0.5 + DIAL_THICKNESS * 0.5, CLOCK_CENTER_Z),
            "lateral_axis": "x",
        }
    if face == "south":
        return {
            "axis": (0.0, -1.0, 0.0),
            "kind": "y",
            "sign": -1.0,
            "cyl_rpy": (math.pi / 2.0, 0.0, 0.0),
            "origin": (0.0, -(UPPER_WALL_DEPTH * 0.5 + DIAL_THICKNESS), CLOCK_CENTER_Z),
            "dial_center": (0.0, -(UPPER_WALL_DEPTH * 0.5 + DIAL_THICKNESS * 0.5), CLOCK_CENTER_Z),
            "lateral_axis": "x",
        }
    if face == "east":
        return {
            "axis": (1.0, 0.0, 0.0),
            "kind": "x",
            "sign": 1.0,
            "cyl_rpy": (0.0, math.pi / 2.0, 0.0),
            "origin": (UPPER_WALL_WIDTH * 0.5 + DIAL_THICKNESS, 0.0, CLOCK_CENTER_Z),
            "dial_center": (UPPER_WALL_WIDTH * 0.5 + DIAL_THICKNESS * 0.5, 0.0, CLOCK_CENTER_Z),
            "lateral_axis": "y",
        }
    if face == "west":
        return {
            "axis": (-1.0, 0.0, 0.0),
            "kind": "x",
            "sign": -1.0,
            "cyl_rpy": (0.0, -math.pi / 2.0, 0.0),
            "origin": (-(UPPER_WALL_WIDTH * 0.5 + DIAL_THICKNESS), 0.0, CLOCK_CENTER_Z),
            "dial_center": (-(UPPER_WALL_WIDTH * 0.5 + DIAL_THICKNESS * 0.5), 0.0, CLOCK_CENTER_Z),
            "lateral_axis": "y",
        }
    raise ValueError(f"Unknown face: {face}")


def _normal_offset(face: str, outward: float, vertical: float, lateral: float = 0.0) -> tuple[float, float, float]:
    spec = _face_spec(face)
    sign = float(spec["sign"])
    if spec["kind"] == "y":
        return (lateral, sign * outward, vertical)
    return (sign * outward, lateral, vertical)


def _face_box_size(face: str, lateral: float, normal: float, vertical: float) -> tuple[float, float, float]:
    if _face_spec(face)["kind"] == "y":
        return (lateral, normal, vertical)
    return (normal, lateral, vertical)


def _add_clock_markers(part, face: str, material) -> None:
    spec = _face_spec(face)
    sign = float(spec["sign"])
    marker_normal = 0.004
    marker_center = (
        UPPER_WALL_DEPTH * 0.5 + DIAL_THICKNESS + marker_normal * 0.5
        if spec["kind"] == "y"
        else UPPER_WALL_WIDTH * 0.5 + DIAL_THICKNESS + marker_normal * 0.5
    )
    if spec["kind"] == "y":
        normal_xyz = sign * marker_center
        part.visual(
            Box((0.13, marker_normal, 0.04)),
            origin=Origin(xyz=(0.0, normal_xyz, CLOCK_CENTER_Z + 0.37)),
            material=material,
            name=f"{face}_marker_top",
        )
        part.visual(
            Box((0.13, marker_normal, 0.04)),
            origin=Origin(xyz=(0.0, normal_xyz, CLOCK_CENTER_Z - 0.37)),
            material=material,
            name=f"{face}_marker_bottom",
        )
        part.visual(
            Box((0.04, marker_normal, 0.13)),
            origin=Origin(xyz=(0.37, normal_xyz, CLOCK_CENTER_Z)),
            material=material,
            name=f"{face}_marker_right",
        )
        part.visual(
            Box((0.04, marker_normal, 0.13)),
            origin=Origin(xyz=(-0.37, normal_xyz, CLOCK_CENTER_Z)),
            material=material,
            name=f"{face}_marker_left",
        )
    else:
        normal_xyz = sign * marker_center
        part.visual(
            Box((marker_normal, 0.13, 0.04)),
            origin=Origin(xyz=(normal_xyz, 0.0, CLOCK_CENTER_Z + 0.37)),
            material=material,
            name=f"{face}_marker_top",
        )
        part.visual(
            Box((marker_normal, 0.13, 0.04)),
            origin=Origin(xyz=(normal_xyz, 0.0, CLOCK_CENTER_Z - 0.37)),
            material=material,
            name=f"{face}_marker_bottom",
        )
        part.visual(
            Box((marker_normal, 0.04, 0.13)),
            origin=Origin(xyz=(normal_xyz, 0.37, CLOCK_CENTER_Z)),
            material=material,
            name=f"{face}_marker_right",
        )
        part.visual(
            Box((marker_normal, 0.04, 0.13)),
            origin=Origin(xyz=(normal_xyz, -0.37, CLOCK_CENTER_Z)),
            material=material,
            name=f"{face}_marker_left",
        )


def _build_arbor(part, face: str, metal_material) -> None:
    spec = _face_spec(face)
    cyl_rpy = spec["cyl_rpy"]

    part.visual(
        Cylinder(radius=0.070, length=0.006),
        origin=Origin(xyz=_normal_offset(face, 0.003, 0.0), rpy=cyl_rpy),
        material=metal_material,
        name="mount_flange",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=_normal_offset(face, 0.014, 0.0), rpy=cyl_rpy),
        material=metal_material,
        name="center_sleeve",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=_normal_offset(face, 0.020, 0.0), rpy=cyl_rpy),
        material=metal_material,
        name="outer_cap",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.14, 0.14, 0.06)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )


def _build_hand(part, face: str, *, hand_type: str, material) -> None:
    if hand_type == "hour":
        blade_length = 0.25
        blade_width = 0.070
        tip_length = 0.08
        pad_height = 0.090
        pad_width = 0.050
        tail_length = 0.060
        normal_center = 0.012
        mass = 0.04
    else:
        blade_length = 0.36
        blade_width = 0.046
        tip_length = 0.12
        pad_height = 0.070
        pad_width = 0.034
        tail_length = 0.050
        normal_center = 0.024
        mass = 0.03

    thickness = 0.006
    tip_width = blade_width * 0.38
    blade_center_z = blade_length * 0.5
    tail_center_z = -tail_length * 0.5
    tip_center_z = blade_length + tip_length * 0.5

    part.visual(
        Box(_face_box_size(face, pad_width, thickness, pad_height)),
        origin=Origin(xyz=_normal_offset(face, normal_center, 0.0)),
        material=material,
        name="hub",
    )
    part.visual(
        Box(_face_box_size(face, blade_width, thickness, blade_length)),
        origin=Origin(xyz=_normal_offset(face, normal_center, blade_center_z)),
        material=material,
        name="blade",
    )
    part.visual(
        Box(_face_box_size(face, tip_width, thickness, tip_length)),
        origin=Origin(xyz=_normal_offset(face, normal_center, tip_center_z)),
        material=material,
        name="tip",
    )
    part.visual(
        Box(_face_box_size(face, blade_width * 0.34, thickness, tail_length)),
        origin=Origin(xyz=_normal_offset(face, normal_center, tail_center_z)),
        material=material,
        name="tail",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.10, 0.04, blade_length + tip_length + tail_length)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, (blade_length + tip_length - tail_length) * 0.25)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_frame_clock_tower")

    timber = model.material("timber_oak", rgba=(0.42, 0.29, 0.18, 1.0))
    plaster = model.material("lime_plaster", rgba=(0.89, 0.86, 0.77, 1.0))
    slate = model.material("slate_roof", rgba=(0.23, 0.24, 0.27, 1.0))
    dial_white = model.material("dial_white", rgba=(0.94, 0.93, 0.88, 1.0))
    iron_dark = model.material("wrought_iron", rgba=(0.14, 0.13, 0.12, 1.0))
    brass = model.material("aged_brass", rgba=(0.74, 0.60, 0.24, 1.0))
    stone = model.material("stone_base", rgba=(0.55, 0.53, 0.50, 1.0))

    tower = model.part("tower_shaft")

    tower.visual(
        Box((3.10, 2.50, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=stone,
        name="foundation",
    )
    tower.visual(
        Box((LOWER_WALL_WIDTH, LOWER_WALL_DEPTH, LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT * 0.5)),
        material=plaster,
        name="lower_wall_mass",
    )
    tower.visual(
        Box((UPPER_WALL_WIDTH, UPPER_WALL_DEPTH, UPPER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT + UPPER_HEIGHT * 0.5)),
        material=plaster,
        name="upper_wall_mass",
    )

    post_x = OUTER_WIDTH * 0.5 - 0.10
    post_y = OUTER_DEPTH * 0.5 - 0.10
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.20, 0.20, TOP_OF_SHAFT)),
                origin=Origin(xyz=(x_sign * post_x, y_sign * post_y, TOP_OF_SHAFT * 0.5)),
                material=timber,
                name=f"corner_post_{'p' if x_sign > 0 else 'm'}x_{'p' if y_sign > 0 else 'm'}y",
            )

    lower_center_post_height = LOWER_HEIGHT - 0.80
    tower.visual(
        Box((0.18, OUTER_DEPTH - 0.44, lower_center_post_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.40 + lower_center_post_height * 0.5)),
        material=timber,
        name="lower_center_post_x",
    )
    tower.visual(
        Box((OUTER_WIDTH - 0.44, 0.18, lower_center_post_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.40 + lower_center_post_height * 0.5)),
        material=timber,
        name="lower_center_post_y",
    )

    beam_levels = (0.60, 2.35, 4.20, 6.40, 7.82)
    for level in beam_levels:
        tower.visual(
            Box((OUTER_WIDTH, 0.20, 0.22)),
            origin=Origin(xyz=(0.0, post_y, level)),
            material=timber,
            name=f"north_beam_{int(level * 100)}",
        )
        tower.visual(
            Box((OUTER_WIDTH, 0.20, 0.22)),
            origin=Origin(xyz=(0.0, -post_y, level)),
            material=timber,
            name=f"south_beam_{int(level * 100)}",
        )
        tower.visual(
            Box((0.20, OUTER_DEPTH, 0.22)),
            origin=Origin(xyz=(post_x, 0.0, level)),
            material=timber,
            name=f"east_beam_{int(level * 100)}",
        )
        tower.visual(
            Box((0.20, OUTER_DEPTH, 0.22)),
            origin=Origin(xyz=(-post_x, 0.0, level)),
            material=timber,
            name=f"west_beam_{int(level * 100)}",
        )

    front_x0 = -OUTER_WIDTH * 0.5 + 0.32
    front_x1 = OUTER_WIDTH * 0.5 - 0.32
    side_y0 = -OUTER_DEPTH * 0.5 + 0.32
    side_y1 = OUTER_DEPTH * 0.5 - 0.32
    brace_thickness = 0.12

    for z0, z1 in ((0.60, 2.35), (2.35, 4.20), (4.20, 6.40)):
        _add_beam(
            tower,
            (front_x0, post_y, z0),
            (front_x1, post_y, z1),
            thickness=brace_thickness,
            material=timber,
            name=f"north_brace_a_{int(z0 * 100)}",
        )
        _add_beam(
            tower,
            (front_x1, post_y, z0),
            (front_x0, post_y, z1),
            thickness=brace_thickness,
            material=timber,
            name=f"north_brace_b_{int(z0 * 100)}",
        )
        _add_beam(
            tower,
            (front_x0, -post_y, z0),
            (front_x1, -post_y, z1),
            thickness=brace_thickness,
            material=timber,
            name=f"south_brace_a_{int(z0 * 100)}",
        )
        _add_beam(
            tower,
            (front_x1, -post_y, z0),
            (front_x0, -post_y, z1),
            thickness=brace_thickness,
            material=timber,
            name=f"south_brace_b_{int(z0 * 100)}",
        )
        _add_beam(
            tower,
            (post_x, side_y0, z0),
            (post_x, side_y1, z1),
            thickness=brace_thickness,
            material=timber,
            name=f"east_brace_a_{int(z0 * 100)}",
        )
        _add_beam(
            tower,
            (post_x, side_y1, z0),
            (post_x, side_y0, z1),
            thickness=brace_thickness,
            material=timber,
            name=f"east_brace_b_{int(z0 * 100)}",
        )
        _add_beam(
            tower,
            (-post_x, side_y0, z0),
            (-post_x, side_y1, z1),
            thickness=brace_thickness,
            material=timber,
            name=f"west_brace_a_{int(z0 * 100)}",
        )
        _add_beam(
            tower,
            (-post_x, side_y1, z0),
            (-post_x, side_y0, z1),
            thickness=brace_thickness,
            material=timber,
            name=f"west_brace_b_{int(z0 * 100)}",
        )

    for face in FACES:
        spec = _face_spec(face)
        tower.visual(
            Box(_face_box_size(face, DIAL_SIZE, DIAL_THICKNESS, DIAL_SIZE)),
            origin=Origin(xyz=spec["dial_center"]),
            material=dial_white,
            name=f"{face}_dial",
        )
        _add_clock_markers(tower, face, iron_dark)

    tower.inertial = Inertial.from_geometry(
        Box((3.10, 2.50, TOP_OF_SHAFT)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, TOP_OF_SHAFT * 0.5)),
    )

    roof = model.part("roof")
    roof_width = 3.20
    roof_depth = 2.60
    roof_skin = 0.06
    slope_len_ns = math.sqrt((roof_depth * 0.5) ** 2 + ROOF_HEIGHT**2)
    slope_len_ew = math.sqrt((roof_width * 0.5) ** 2 + ROOF_HEIGHT**2)
    north_slope = math.atan2(ROOF_HEIGHT, roof_depth * 0.5)
    east_slope = math.atan2(ROOF_HEIGHT, roof_width * 0.5)

    roof.visual(
        Box((roof_width, slope_len_ns, roof_skin)),
        origin=Origin(
            xyz=(0.0, roof_depth * 0.25, ROOF_HEIGHT * 0.5),
            rpy=(-north_slope, 0.0, 0.0),
        ),
        material=slate,
        name="north_roof_plane",
    )
    roof.visual(
        Box((roof_width, slope_len_ns, roof_skin)),
        origin=Origin(
            xyz=(0.0, -roof_depth * 0.25, ROOF_HEIGHT * 0.5),
            rpy=(north_slope, 0.0, 0.0),
        ),
        material=slate,
        name="south_roof_plane",
    )
    roof.visual(
        Box((slope_len_ew, roof_depth, roof_skin)),
        origin=Origin(
            xyz=(roof_width * 0.25, 0.0, ROOF_HEIGHT * 0.5),
            rpy=(0.0, east_slope, 0.0),
        ),
        material=slate,
        name="east_roof_plane",
    )
    roof.visual(
        Box((slope_len_ew, roof_depth, roof_skin)),
        origin=Origin(
            xyz=(-roof_width * 0.25, 0.0, ROOF_HEIGHT * 0.5),
            rpy=(0.0, -east_slope, 0.0),
        ),
        material=slate,
        name="west_roof_plane",
    )
    roof.visual(
        Box((0.90, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_HEIGHT - 0.04)),
        material=timber,
        name="ridge_cap",
    )
    roof.visual(
        Box((0.14, 0.14, ROOF_HEIGHT - 0.08)),
        origin=Origin(xyz=(0.0, 0.0, (ROOF_HEIGHT - 0.08) * 0.5)),
        material=timber,
        name="king_post",
    )
    roof.inertial = Inertial.from_geometry(
        Box((3.20, 2.60, ROOF_HEIGHT)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, ROOF_HEIGHT * 0.5)),
    )

    model.articulation(
        "tower_to_roof",
        ArticulationType.FIXED,
        parent=tower,
        child=roof,
        origin=Origin(xyz=(0.0, 0.0, TOP_OF_SHAFT)),
    )

    hand_limits = MotionLimits(
        effort=0.5,
        velocity=1.5,
        lower=-2.0 * math.pi,
        upper=2.0 * math.pi,
    )

    for face in FACES:
        spec = _face_spec(face)

        arbor = model.part(f"{face}_arbor")
        _build_arbor(arbor, face, brass)
        model.articulation(
            f"tower_to_{face}_arbor",
            ArticulationType.FIXED,
            parent=tower,
            child=arbor,
            origin=Origin(xyz=spec["origin"]),
        )

        hour_hand = model.part(f"{face}_hour_hand")
        _build_hand(hour_hand, face, hand_type="hour", material=iron_dark)
        model.articulation(
            f"{face}_hour_joint",
            ArticulationType.REVOLUTE,
            parent=arbor,
            child=hour_hand,
            origin=Origin(),
            axis=spec["axis"],
            motion_limits=hand_limits,
        )

        minute_hand = model.part(f"{face}_minute_hand")
        _build_hand(minute_hand, face, hand_type="minute", material=brass)
        model.articulation(
            f"{face}_minute_joint",
            ArticulationType.REVOLUTE,
            parent=arbor,
            child=minute_hand,
            origin=Origin(),
            axis=spec["axis"],
            motion_limits=hand_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    tower = object_model.get_part("tower_shaft")
    roof = object_model.get_part("roof")

    for face in FACES:
        arbor = object_model.get_part(f"{face}_arbor")
        hour_hand = object_model.get_part(f"{face}_hour_hand")
        ctx.allow_overlap(
            arbor,
            hour_hand,
            reason="The hour hand hub is intentionally sleeved over the central spindle for a concentric clock-hand mount.",
        )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(roof, tower, name="roof_contacts_tower")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_adjacent=False,
        ignore_fixed=True,
    )
    ctx.fail_if_isolated_parts(max_pose_samples=16, name="sampled_pose_no_floating")

    tower_aabb = ctx.part_world_aabb(tower)
    roof_aabb = ctx.part_world_aabb(roof)
    assert tower_aabb is not None
    assert roof_aabb is not None
    total_height = roof_aabb[1][2] - tower_aabb[0][2]
    ctx.check(
        "clock_tower_total_height_realistic",
        8.8 <= total_height <= 10.2,
        f"Expected total height around 9-10 m, got {total_height:.3f} m.",
    )

    def _axis_extent(aabb, axis: str) -> float:
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[1][axis_index] - aabb[0][axis_index]

    for face in FACES:
        spec = _face_spec(face)
        arbor = object_model.get_part(f"{face}_arbor")
        hour_hand = object_model.get_part(f"{face}_hour_hand")
        minute_hand = object_model.get_part(f"{face}_minute_hand")
        hour_joint = object_model.get_articulation(f"{face}_hour_joint")
        minute_joint = object_model.get_articulation(f"{face}_minute_joint")

        ctx.expect_contact(arbor, tower, name=f"{face}_arbor_contacts_tower")
        ctx.expect_contact(hour_hand, arbor, name=f"{face}_hour_contacts_arbor")
        ctx.expect_contact(minute_hand, arbor, name=f"{face}_minute_contacts_arbor")

        ctx.check(
            f"{face}_hour_axis_correct",
            tuple(hour_joint.axis) == tuple(spec["axis"]),
            f"Expected {face} hour axis {spec['axis']}, got {hour_joint.axis}.",
        )
        ctx.check(
            f"{face}_minute_axis_correct",
            tuple(minute_joint.axis) == tuple(spec["axis"]),
            f"Expected {face} minute axis {spec['axis']}, got {minute_joint.axis}.",
        )

        with ctx.pose({hour_joint: 0.0, minute_joint: 0.0}):
            ctx.expect_contact(hour_hand, arbor, name=f"{face}_hour_rest_contact")
            ctx.expect_contact(minute_hand, arbor, name=f"{face}_minute_rest_contact")

        for joint in (hour_joint, minute_joint):
            limits = joint.motion_limits
            assert limits is not None
            assert limits.lower is not None
            assert limits.upper is not None
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

        transverse_axis = str(spec["lateral_axis"])
        for hand_part, joint in ((hour_hand, hour_joint), (minute_hand, minute_joint)):
            with ctx.pose({joint: 0.0}):
                rest_blade_aabb = ctx.part_element_world_aabb(hand_part, elem="blade")
            with ctx.pose({joint: math.pi / 2.0}):
                turned_blade_aabb = ctx.part_element_world_aabb(hand_part, elem="blade")

            assert rest_blade_aabb is not None
            assert turned_blade_aabb is not None
            rest_transverse = _axis_extent(rest_blade_aabb, transverse_axis)
            turned_transverse = _axis_extent(turned_blade_aabb, transverse_axis)
            rest_vertical = _axis_extent(rest_blade_aabb, "z")
            turned_vertical = _axis_extent(turned_blade_aabb, "z")

            ctx.check(
                f"{joint.name}_turns_across_face",
                turned_transverse > rest_transverse + 0.10 and turned_vertical < rest_vertical - 0.10,
                (
                    f"{joint.name} did not rotate from vertical to transverse enough: "
                    f"rest transverse={rest_transverse:.3f}, turned transverse={turned_transverse:.3f}, "
                    f"rest vertical={rest_vertical:.3f}, turned vertical={turned_vertical:.3f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
