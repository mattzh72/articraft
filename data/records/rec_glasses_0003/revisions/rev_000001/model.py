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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _temple_wire_mesh(name: str, side_sign: float):
    return _save_mesh(
        name,
        tube_from_spline_points(
            [
                (0.0, -0.0055, 0.0),
                (side_sign * 0.0035, -0.030, 0.0010),
                (side_sign * 0.0055, -0.082, -0.0030),
                (side_sign * 0.0030, -0.122, -0.0160),
                (-side_sign * 0.0030, -0.138, -0.0310),
            ],
            radius=0.00135,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )


def _nose_pad_bracket_mesh(name: str, side_sign: float):
    return _save_mesh(
        name,
        tube_from_spline_points(
            [
                (side_sign * 0.0075, -0.0006, 0.0005),
                (side_sign * 0.0065, -0.0030, -0.0040),
                (side_sign * 0.0070, -0.0060, -0.0080),
            ],
            radius=0.0008,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
    )


def _bridge_mesh(name: str):
    return _save_mesh(
        name,
        tube_from_spline_points(
            [
                (-0.0120, -0.0002, 0.0038),
                (-0.0065, 0.0006, 0.0082),
                (0.0, 0.0009, 0.0096),
                (0.0065, 0.0006, 0.0082),
                (0.0120, -0.0002, 0.0038),
            ],
            radius=0.00125,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )


def _endpiece_link_mesh(name: str, side_sign: float):
    return _save_mesh(
        name,
        tube_from_spline_points(
            [
                (side_sign * 0.0560, -0.0003, 0.0035),
                (side_sign * 0.0595, -0.0008, 0.0042),
                (side_sign * 0.0612, -0.0010, 0.0040),
            ],
            radius=0.00125,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        ),
    )


def _visual_center_from_aabb(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_wire_glasses", assets=ASSETS)

    metal = model.material("polished_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_metal = model.material("hinge_metal", rgba=(0.43, 0.45, 0.48, 1.0))
    lens_clear = model.material("clear_lens", rgba=(0.84, 0.90, 0.96, 0.18))
    pad_clear = model.material("nose_pad", rgba=(0.92, 0.92, 0.92, 0.70))

    lens_radius = 0.024
    wire_radius = 0.0013
    lens_thickness = 0.0012
    lens_center_x = 0.033
    hinge_x = 0.061
    barrel_radius = 0.00185
    frame_barrel_length = 0.0025
    temple_barrel_length = 0.0050
    barrel_offset_z = 0.00375

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.128, 0.012, 0.055)),
        mass=0.030,
        origin=Origin(xyz=(0.0, -0.001, 0.0)),
    )

    rim_mesh = _save_mesh(
        "round_rim.obj",
        TorusGeometry(
            radius=lens_radius,
            tube=wire_radius,
            radial_segments=18,
            tubular_segments=64,
        ).rotate_x(math.pi / 2.0),
    )
    bridge_mesh = _bridge_mesh("bridge_wire.obj")
    left_bracket_mesh = _nose_pad_bracket_mesh("left_nose_bracket.obj", -1.0)
    right_bracket_mesh = _nose_pad_bracket_mesh("right_nose_bracket.obj", 1.0)
    left_endpiece_mesh = _endpiece_link_mesh("left_endpiece_link.obj", -1.0)
    right_endpiece_mesh = _endpiece_link_mesh("right_endpiece_link.obj", 1.0)

    front_frame.visual(
        rim_mesh,
        origin=Origin(xyz=(-lens_center_x, 0.0, 0.0)),
        material=metal,
        name="left_rim",
    )
    front_frame.visual(
        rim_mesh,
        origin=Origin(xyz=(lens_center_x, 0.0, 0.0)),
        material=metal,
        name="right_rim",
    )
    front_frame.visual(
        Cylinder(radius=lens_radius - wire_radius * 1.25, length=lens_thickness),
        origin=Origin(xyz=(-lens_center_x, -0.0006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_clear,
        name="left_lens",
    )
    front_frame.visual(
        Cylinder(radius=lens_radius - wire_radius * 1.25, length=lens_thickness),
        origin=Origin(xyz=(lens_center_x, -0.0006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_clear,
        name="right_lens",
    )
    front_frame.visual(bridge_mesh, material=metal, name="bridge_wire")
    front_frame.visual(left_bracket_mesh, material=metal, name="left_pad_bracket")
    front_frame.visual(right_bracket_mesh, material=metal, name="right_pad_bracket")
    front_frame.visual(
        Cylinder(radius=0.0033, length=0.0017),
        origin=Origin(xyz=(-0.0070, -0.0068, -0.0090), rpy=(math.pi / 2.0, 0.0, -0.28)),
        material=pad_clear,
        name="left_nose_pad",
    )
    front_frame.visual(
        Cylinder(radius=0.0033, length=0.0017),
        origin=Origin(xyz=(0.0070, -0.0068, -0.0090), rpy=(math.pi / 2.0, 0.0, 0.28)),
        material=pad_clear,
        name="right_nose_pad",
    )
    front_frame.visual(left_endpiece_mesh, material=metal, name="left_endpiece_wire")
    front_frame.visual(right_endpiece_mesh, material=metal, name="right_endpiece_wire")
    front_frame.visual(
        Box((0.0046, 0.0026, 0.0110)),
        origin=Origin(xyz=(-0.0592, -0.0010, 0.0)),
        material=dark_metal,
        name="left_hinge_block",
    )
    front_frame.visual(
        Box((0.0046, 0.0026, 0.0110)),
        origin=Origin(xyz=(0.0592, -0.0010, 0.0)),
        material=dark_metal,
        name="right_hinge_block",
    )
    front_frame.visual(
        Cylinder(radius=barrel_radius, length=frame_barrel_length),
        origin=Origin(xyz=(-hinge_x, -0.0011, barrel_offset_z)),
        material=dark_metal,
        name="left_frame_upper_barrel",
    )
    front_frame.visual(
        Cylinder(radius=barrel_radius, length=frame_barrel_length),
        origin=Origin(xyz=(-hinge_x, -0.0011, -barrel_offset_z)),
        material=dark_metal,
        name="left_frame_lower_barrel",
    )
    front_frame.visual(
        Cylinder(radius=barrel_radius, length=frame_barrel_length),
        origin=Origin(xyz=(hinge_x, -0.0011, barrel_offset_z)),
        material=dark_metal,
        name="right_frame_upper_barrel",
    )
    front_frame.visual(
        Cylinder(radius=barrel_radius, length=frame_barrel_length),
        origin=Origin(xyz=(hinge_x, -0.0011, -barrel_offset_z)),
        material=dark_metal,
        name="right_frame_lower_barrel",
    )

    left_temple = model.part("left_temple")
    left_temple.inertial = Inertial.from_geometry(
        Box((0.014, 0.145, 0.040)),
        mass=0.010,
        origin=Origin(xyz=(0.0, -0.072, -0.012)),
    )
    left_temple.visual(
        Cylinder(radius=0.00178, length=temple_barrel_length),
        origin=Origin(xyz=(0.0, -0.0011, 0.0)),
        material=dark_metal,
        name="left_center_barrel",
    )
    left_temple.visual(
        Box((0.0036, 0.0064, 0.0105)),
        origin=Origin(xyz=(-0.0010, -0.0042, 0.0)),
        material=dark_metal,
        name="left_hinge_leaf",
    )
    left_temple.visual(
        _temple_wire_mesh("left_temple_wire.obj", -1.0),
        material=metal,
        name="left_temple_wire",
    )
    left_temple.visual(
        Sphere(radius=0.0018),
        origin=Origin(xyz=(0.0030, -0.1380, -0.0310)),
        material=metal,
        name="left_tip",
    )

    right_temple = model.part("right_temple")
    right_temple.inertial = Inertial.from_geometry(
        Box((0.014, 0.145, 0.040)),
        mass=0.010,
        origin=Origin(xyz=(0.0, -0.072, -0.012)),
    )
    right_temple.visual(
        Cylinder(radius=0.00178, length=temple_barrel_length),
        origin=Origin(xyz=(0.0, -0.0011, 0.0)),
        material=dark_metal,
        name="right_center_barrel",
    )
    right_temple.visual(
        Box((0.0036, 0.0064, 0.0105)),
        origin=Origin(xyz=(0.0010, -0.0042, 0.0)),
        material=dark_metal,
        name="right_hinge_leaf",
    )
    right_temple.visual(
        _temple_wire_mesh("right_temple_wire.obj", 1.0),
        material=metal,
        name="right_temple_wire",
    )
    right_temple.visual(
        Sphere(radius=0.0018),
        origin=Origin(xyz=(-0.0030, -0.1380, -0.0310)),
        material=metal,
        name="right_tip",
    )

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(-hinge_x, -0.0011, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(hinge_x, -0.0011, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    front_frame = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")

    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")

    left_frame_upper_barrel = front_frame.get_visual("left_frame_upper_barrel")
    left_frame_lower_barrel = front_frame.get_visual("left_frame_lower_barrel")
    right_frame_upper_barrel = front_frame.get_visual("right_frame_upper_barrel")
    right_frame_lower_barrel = front_frame.get_visual("right_frame_lower_barrel")
    left_center_barrel = left_temple.get_visual("left_center_barrel")
    right_center_barrel = right_temple.get_visual("right_center_barrel")
    left_tip = left_temple.get_visual("left_tip")
    right_tip = right_temple.get_visual("right_tip")
    bridge_wire = front_frame.get_visual("bridge_wire")
    left_nose_pad = front_frame.get_visual("left_nose_pad")
    right_nose_pad = front_frame.get_visual("right_nose_pad")
    left_lens = front_frame.get_visual("left_lens")
    right_lens = front_frame.get_visual("right_lens")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    ctx.expect_contact(
        left_temple,
        front_frame,
        elem_a=left_center_barrel,
        elem_b=left_frame_upper_barrel,
        name="left upper hinge barrel contact",
    )
    ctx.expect_contact(
        left_temple,
        front_frame,
        elem_a=left_center_barrel,
        elem_b=left_frame_lower_barrel,
        name="left lower hinge barrel contact",
    )
    ctx.expect_contact(
        right_temple,
        front_frame,
        elem_a=right_center_barrel,
        elem_b=right_frame_upper_barrel,
        name="right upper hinge barrel contact",
    )
    ctx.expect_contact(
        right_temple,
        front_frame,
        elem_a=right_center_barrel,
        elem_b=right_frame_lower_barrel,
        name="right lower hinge barrel contact",
    )
    ctx.expect_overlap(
        left_temple,
        front_frame,
        axes="xy",
        elem_a=left_center_barrel,
        elem_b=left_frame_upper_barrel,
        min_overlap=0.0024,
        name="left hinge barrel footprint overlap",
    )
    ctx.expect_overlap(
        right_temple,
        front_frame,
        axes="xy",
        elem_a=right_center_barrel,
        elem_b=right_frame_upper_barrel,
        min_overlap=0.0024,
        name="right hinge barrel footprint overlap",
    )

    front_aabb = ctx.part_world_aabb(front_frame)
    front_ok = front_aabb is not None
    if front_aabb is not None:
        (min_x, min_y, min_z), (max_x, max_y, max_z) = front_aabb
        frame_width = max_x - min_x
        frame_height = max_z - min_z
        frame_depth = max_y - min_y
        front_ok = (
            0.120 <= frame_width <= 0.135
            and 0.046 <= frame_height <= 0.055
            and frame_depth <= 0.016
        )
    ctx.check(
        "front frame proportions",
        front_ok,
        details=f"front_frame aabb={front_aabb}",
    )

    left_lens_aabb = ctx.part_element_world_aabb(front_frame, elem=left_lens)
    right_lens_aabb = ctx.part_element_world_aabb(front_frame, elem=right_lens)
    lens_centers = (
        _visual_center_from_aabb(left_lens_aabb),
        _visual_center_from_aabb(right_lens_aabb),
    )
    lens_ok = False
    if lens_centers[0] is not None and lens_centers[1] is not None:
        lens_spacing = lens_centers[1][0] - lens_centers[0][0]
        lens_ok = 0.060 <= lens_spacing <= 0.068 and abs(lens_centers[0][2] - lens_centers[1][2]) <= 0.001
    ctx.check(
        "round lenses sit in a symmetric pair",
        lens_ok,
        details=f"left_lens={left_lens_aabb}, right_lens={right_lens_aabb}",
    )

    bridge_aabb = ctx.part_element_world_aabb(front_frame, elem=bridge_wire)
    left_pad_aabb = ctx.part_element_world_aabb(front_frame, elem=left_nose_pad)
    right_pad_aabb = ctx.part_element_world_aabb(front_frame, elem=right_nose_pad)
    bridge_center = _visual_center_from_aabb(bridge_aabb)
    left_pad_center = _visual_center_from_aabb(left_pad_aabb)
    right_pad_center = _visual_center_from_aabb(right_pad_aabb)
    pads_ok = False
    if bridge_center and left_pad_center and right_pad_center:
        pads_ok = (
            left_pad_center[1] < bridge_center[1] - 0.004
            and right_pad_center[1] < bridge_center[1] - 0.004
            and left_pad_center[2] < bridge_center[2] - 0.010
            and right_pad_center[2] < bridge_center[2] - 0.010
            and left_pad_center[0] < 0.0 < right_pad_center[0]
        )
    ctx.check(
        "nose pads hang below and behind bridge",
        pads_ok,
        details=f"bridge={bridge_aabb}, left_pad={left_pad_aabb}, right_pad={right_pad_aabb}",
    )

    left_tip_rest = _visual_center_from_aabb(ctx.part_element_world_aabb(left_temple, elem=left_tip))
    right_tip_rest = _visual_center_from_aabb(ctx.part_element_world_aabb(right_temple, elem=right_tip))
    left_open_ok = left_tip_rest is not None and left_tip_rest[1] < -0.120 and left_tip_rest[0] < -0.055
    right_open_ok = right_tip_rest is not None and right_tip_rest[1] < -0.120 and right_tip_rest[0] > 0.055
    ctx.check(
        "temple arms extend rearward in open pose",
        left_open_ok and right_open_ok,
        details=f"left_tip_rest={left_tip_rest}, right_tip_rest={right_tip_rest}",
    )

    with ctx.pose({left_hinge: math.radians(88.0), right_hinge: math.radians(88.0)}):
        left_tip_folded = _visual_center_from_aabb(ctx.part_element_world_aabb(left_temple, elem=left_tip))
        right_tip_folded = _visual_center_from_aabb(ctx.part_element_world_aabb(right_temple, elem=right_tip))
        folded_ok = False
        if left_tip_rest and right_tip_rest and left_tip_folded and right_tip_folded:
            folded_ok = (
                left_tip_folded[0] > 0.045
                and right_tip_folded[0] < -0.045
                and abs(left_tip_folded[1]) < 0.020
                and abs(right_tip_folded[1]) < 0.020
                and abs(left_tip_folded[2] - left_tip_rest[2]) < 0.002
                and abs(right_tip_folded[2] - right_tip_rest[2]) < 0.002
            )
        ctx.check(
            "hinges fold temples inward about ninety degrees",
            folded_ok,
            details=f"left_folded={left_tip_folded}, right_folded={right_tip_folded}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
