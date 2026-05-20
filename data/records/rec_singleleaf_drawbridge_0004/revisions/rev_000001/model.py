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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_aligned_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_cylinder_member(
    part,
    *,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_aligned_member(a, b)),
        material=material,
        name=name,
    )


def _add_bolt_on_y_face(
    part,
    *,
    x: float,
    y_face: float,
    z: float,
    radius: float,
    head_len: float,
    embed: float,
    material,
    name: str,
) -> None:
    sign = 1.0 if y_face >= 0.0 else -1.0
    part.visual(
        Cylinder(radius=radius, length=head_len),
        origin=Origin(
            xyz=(x, y_face + sign * (0.5 * head_len - embed), z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_bolt_on_z_face(
    part,
    *,
    x: float,
    y: float,
    z_face: float,
    radius: float,
    head_len: float,
    embed: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=head_len),
        origin=Origin(xyz=(x, y, z_face + 0.5 * head_len - embed)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="singleleaf_drawbridge", assets=ASSETS)

    weathered_concrete = model.material(
        "weathered_concrete", rgba=(0.57, 0.58, 0.56, 1.0)
    )
    bridge_blue = model.material("bridge_blue", rgba=(0.18, 0.27, 0.33, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.76, 0.63, 0.14, 1.0))
    deck_tread = model.material("deck_tread", rgba=(0.17, 0.18, 0.19, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.48, 0.49, 0.51, 1.0))
    wear_pad = model.material("wear_pad", rgba=(0.09, 0.09, 0.10, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((4.60, 4.40, 1.10)),
        origin=Origin(xyz=(-0.90, 0.0, 0.55)),
        material=weathered_concrete,
        name="foundation_plinth",
    )
    support_frame.visual(
        Box((1.60, 3.80, 1.80)),
        origin=Origin(xyz=(-2.10, 0.0, 1.55)),
        material=weathered_concrete,
        name="rear_abutment_block",
    )
    support_frame.visual(
        Box((1.40, 2.00, 0.90)),
        origin=Origin(xyz=(-1.00, 0.0, 1.15)),
        material=bridge_blue,
        name="machinery_plinth",
    )
    support_frame.visual(
        Box((1.20, 1.70, 0.05)),
        origin=Origin(xyz=(-0.95, 0.0, 1.625)),
        material=deck_tread,
        name="service_deck",
    )
    support_frame.visual(
        Box((0.90, 0.90, 0.60)),
        origin=Origin(xyz=(-0.95, 0.0, 1.95)),
        material=bridge_blue,
        name="power_pack_housing",
    )
    support_frame.visual(
        Box((2.00, 2.40, 0.12)),
        origin=Origin(xyz=(-0.55, 0.0, 1.18)),
        material=bridge_blue,
        name="saddle_plate",
    )
    support_frame.visual(
        Box((0.90, 1.80, 0.24)),
        origin=Origin(xyz=(0.75, 0.0, 1.20)),
        material=bridge_blue,
        name="front_nose_sill",
    )
    support_frame.visual(
        Box((1.00, 0.18, 1.46)),
        origin=Origin(xyz=(-0.05, 1.92, 1.83)),
        material=bridge_blue,
        name="tower_left_web",
    )
    support_frame.visual(
        Box((1.00, 0.18, 1.46)),
        origin=Origin(xyz=(-0.05, -1.92, 1.83)),
        material=bridge_blue,
        name="tower_right_web",
    )
    support_frame.visual(
        Box((1.40, 3.60, 0.24)),
        origin=Origin(xyz=(-1.00, 0.0, 2.18)),
        material=bridge_blue,
        name="rear_crossbeam",
    )
    support_frame.visual(
        Box((0.90, 0.34, 0.04)),
        origin=Origin(xyz=(0.60, 1.42, 1.30)),
        material=wear_pad,
        name="support_seat_left",
    )
    support_frame.visual(
        Box((0.90, 0.34, 0.04)),
        origin=Origin(xyz=(0.60, -1.42, 1.30)),
        material=wear_pad,
        name="support_seat_right",
    )
    support_frame.visual(
        Box((0.90, 0.36, 0.18)),
        origin=Origin(xyz=(0.60, 1.08, 1.21)),
        material=bridge_blue,
        name="seat_bracket_left",
    )
    support_frame.visual(
        Box((0.90, 0.36, 0.18)),
        origin=Origin(xyz=(0.60, -1.08, 1.21)),
        material=bridge_blue,
        name="seat_bracket_right",
    )
    _add_cylinder_member(
        support_frame,
        a=(0.15, 1.82, 1.28),
        b=(1.00, 1.82, 1.60),
        radius=0.085,
        material=wear_pad,
        name="support_bumper_left",
    )
    _add_cylinder_member(
        support_frame,
        a=(0.15, -1.82, 1.28),
        b=(1.00, -1.82, 1.60),
        radius=0.085,
        material=wear_pad,
        name="support_bumper_right",
    )
    _add_cylinder_member(
        support_frame,
        a=(-1.20, 1.92, 1.10),
        b=(0.15, 1.92, 2.38),
        radius=0.065,
        material=hardware_steel,
        name="tower_brace_left",
    )
    _add_cylinder_member(
        support_frame,
        a=(-1.20, -1.92, 1.10),
        b=(0.15, -1.92, 2.38),
        radius=0.065,
        material=hardware_steel,
        name="tower_brace_right",
    )
    support_frame.visual(
        Box((0.34, 0.46, 0.52)),
        origin=Origin(xyz=(-0.38, 1.76, 1.78)),
        material=hardware_steel,
        name="support_bearing_housing_left",
    )
    support_frame.visual(
        Box((0.34, 0.46, 0.52)),
        origin=Origin(xyz=(-0.38, -1.76, 1.78)),
        material=hardware_steel,
        name="support_bearing_housing_right",
    )
    support_frame.visual(
        Cylinder(radius=0.10, length=0.62),
        origin=Origin(
            xyz=(0.0, 1.70, 1.78), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=hardware_steel,
        name="support_hinge_pin_left",
    )
    support_frame.visual(
        Cylinder(radius=0.10, length=0.62),
        origin=Origin(
            xyz=(0.0, -1.70, 1.78), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=hardware_steel,
        name="support_hinge_pin_right",
    )
    support_frame.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(
            xyz=(0.0, 2.05, 1.78), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=hardware_steel,
        name="support_pin_cap_left",
    )
    support_frame.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(
            xyz=(0.0, -2.05, 1.78), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=hardware_steel,
        name="support_pin_cap_right",
    )
    for side_name, y_face in (("left", 2.01), ("right", -2.01)):
        for index, (x, z) in enumerate(
            (
                (-0.18, 1.56),
                (0.18, 1.56),
                (-0.18, 2.00),
                (0.18, 2.00),
                (-0.18, 1.78),
                (0.18, 1.78),
            )
        ):
            _add_bolt_on_y_face(
                support_frame,
                x=x,
                y_face=y_face,
                z=z,
                radius=0.020,
                head_len=0.05,
                embed=0.008,
                material=hardware_steel,
                name=f"bearing_bolt_{side_name}_{index}",
            )
    for index, (x, y) in enumerate(
        ((-1.20, 0.70), (-1.20, -0.70), (-0.60, 0.70), (-0.60, -0.70))
    ):
        _add_bolt_on_z_face(
            support_frame,
            x=x,
            y=y,
            z_face=1.60,
            radius=0.028,
            head_len=0.10,
            embed=0.01,
            material=hardware_steel,
            name=f"anchor_bolt_{index}",
        )
    support_frame.inertial = Inertial.from_geometry(
        Box((4.60, 4.40, 2.50)),
        mass=145000.0,
        origin=Origin(xyz=(-0.90, 0.0, 1.25)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((0.95, 2.40, 0.72)),
        origin=Origin(xyz=(0.32, 0.0, -0.08)),
        material=bridge_blue,
        name="hinge_torque_beam",
    )
    bridge_leaf.visual(
        Box((7.50, 0.24, 0.56)),
        origin=Origin(xyz=(4.00, 1.46, -0.18)),
        material=safety_yellow,
        name="leaf_girder_left",
    )
    bridge_leaf.visual(
        Box((7.50, 0.24, 0.56)),
        origin=Origin(xyz=(4.00, -1.46, -0.18)),
        material=safety_yellow,
        name="leaf_girder_right",
    )
    bridge_leaf.visual(
        Box((1.10, 3.10, 0.05)),
        origin=Origin(xyz=(0.55, 0.0, 0.125)),
        material=safety_yellow,
        name="root_deck_plate",
    )
    bridge_leaf.visual(
        Box((6.90, 3.10, 0.06)),
        origin=Origin(xyz=(4.05, 0.0, 0.13)),
        material=safety_yellow,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((0.80, 1.10, 0.012)),
        origin=Origin(xyz=(1.20, 0.0, 0.166)),
        material=deck_tread,
        name="access_cover",
    )
    bridge_leaf.visual(
        Box((0.30, 3.60, 0.44)),
        origin=Origin(xyz=(7.90, 0.0, -0.12)),
        material=bridge_blue,
        name="nose_beam",
    )
    for index, x in enumerate((1.25, 2.40, 3.55, 4.70, 5.85, 7.00)):
        bridge_leaf.visual(
            Box((0.14, 2.68, 0.38)),
            origin=Origin(xyz=(x, 0.0, -0.16)),
            material=bridge_blue,
            name=f"cross_rib_{index}",
        )
    bridge_leaf.visual(
        Box((6.80, 0.18, 0.16)),
        origin=Origin(xyz=(4.05, 1.42, 0.21)),
        material=bridge_blue,
        name="curb_left",
    )
    bridge_leaf.visual(
        Box((6.80, 0.18, 0.16)),
        origin=Origin(xyz=(4.05, -1.42, 0.21)),
        material=bridge_blue,
        name="curb_right",
    )
    bridge_leaf.visual(
        Box((6.40, 0.55, 0.02)),
        origin=Origin(xyz=(4.10, 0.78, 0.17)),
        material=deck_tread,
        name="wear_strip_left",
    )
    bridge_leaf.visual(
        Box((6.40, 0.55, 0.02)),
        origin=Origin(xyz=(4.10, -0.78, 0.17)),
        material=deck_tread,
        name="wear_strip_right",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.17, length=0.44),
        origin=Origin(
            xyz=(0.0, 1.61, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=hardware_steel,
        name="leaf_hinge_sleeve_left",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.17, length=0.44),
        origin=Origin(
            xyz=(0.0, -1.61, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=hardware_steel,
        name="leaf_hinge_sleeve_right",
    )
    bridge_leaf.visual(
        Box((0.18, 0.10, 0.52)),
        origin=Origin(xyz=(0.28, 1.45, -0.04)),
        material=bridge_blue,
        name="hinge_cheek_left",
    )
    bridge_leaf.visual(
        Box((0.18, 0.10, 0.52)),
        origin=Origin(xyz=(0.28, -1.45, -0.04)),
        material=bridge_blue,
        name="hinge_cheek_right",
    )
    bridge_leaf.visual(
        Box((0.44, 0.08, 0.18)),
        origin=Origin(xyz=(0.44, 1.46, 0.18), rpy=(0.0, -0.68, 0.0)),
        material=bridge_blue,
        name="hinge_gusset_upper_left",
    )
    bridge_leaf.visual(
        Box((0.44, 0.08, 0.18)),
        origin=Origin(xyz=(0.44, -1.46, 0.18), rpy=(0.0, -0.68, 0.0)),
        material=bridge_blue,
        name="hinge_gusset_upper_right",
    )
    bridge_leaf.visual(
        Box((0.38, 0.08, 0.16)),
        origin=Origin(xyz=(0.36, 1.46, -0.28), rpy=(0.0, 0.58, 0.0)),
        material=bridge_blue,
        name="hinge_gusset_lower_left",
    )
    bridge_leaf.visual(
        Box((0.38, 0.08, 0.16)),
        origin=Origin(xyz=(0.36, -1.46, -0.28), rpy=(0.0, 0.58, 0.0)),
        material=bridge_blue,
        name="hinge_gusset_lower_right",
    )
    bridge_leaf.visual(
        Box((3.20, 0.08, 0.24)),
        origin=Origin(xyz=(2.55, 0.74, -0.02), rpy=(0.0, 0.0, 0.38)),
        material=bridge_blue,
        name="deck_diagonal_left",
    )
    bridge_leaf.visual(
        Box((3.20, 0.08, 0.24)),
        origin=Origin(xyz=(2.55, -0.74, -0.02), rpy=(0.0, 0.0, -0.38)),
        material=bridge_blue,
        name="deck_diagonal_right",
    )
    bolt_positions = (1.20, 2.40, 3.60, 4.80, 6.00, 7.20)
    for side_name, y in (("left", 0.72), ("right", -0.72)):
        for index, x in enumerate(bolt_positions):
            _add_bolt_on_z_face(
                bridge_leaf,
                x=x,
                y=y,
                z_face=0.16,
                radius=0.018,
                head_len=0.05,
                embed=0.008,
                material=hardware_steel,
                name=f"deck_bolt_{side_name}_{index}",
            )
    for side_name, y_face in (("left", 1.58), ("right", -1.58)):
        for index, x in enumerate((0.90, 2.60, 4.30, 6.00, 7.60)):
            _add_bolt_on_y_face(
                bridge_leaf,
                x=x,
                y_face=y_face,
                z=-0.10,
                radius=0.016,
                head_len=0.05,
                embed=0.008,
                material=hardware_steel,
                name=f"girder_bolt_{side_name}_{index}",
            )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((8.10, 3.60, 0.90)),
        mass=42000.0,
        origin=Origin(xyz=(4.00, 0.0, -0.05)),
    )

    model.articulation(
        "support_to_bridge_leaf",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.78)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900000.0,
            velocity=0.18,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support_frame = object_model.get_part("support_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_hinge = object_model.get_articulation("support_to_bridge_leaf")

    support_seat_left = support_frame.get_visual("support_seat_left")
    support_seat_right = support_frame.get_visual("support_seat_right")
    tower_left_web = support_frame.get_visual("tower_left_web")
    tower_right_web = support_frame.get_visual("tower_right_web")
    support_pin_left = support_frame.get_visual("support_hinge_pin_left")
    support_pin_right = support_frame.get_visual("support_hinge_pin_right")
    saddle_plate = support_frame.get_visual("saddle_plate")
    housing_left = support_frame.get_visual("support_bearing_housing_left")
    housing_right = support_frame.get_visual("support_bearing_housing_right")

    leaf_girder_left = bridge_leaf.get_visual("leaf_girder_left")
    leaf_girder_right = bridge_leaf.get_visual("leaf_girder_right")
    deck_plate = bridge_leaf.get_visual("deck_plate")
    root_deck_plate = bridge_leaf.get_visual("root_deck_plate")
    nose_beam = bridge_leaf.get_visual("nose_beam")
    sleeve_left = bridge_leaf.get_visual("leaf_hinge_sleeve_left")
    sleeve_right = bridge_leaf.get_visual("leaf_hinge_sleeve_right")

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
    ctx.allow_overlap(
        support_frame,
        bridge_leaf,
        elem_a=support_pin_left,
        elem_b=sleeve_left,
        reason="Left hinge pin nests inside the leaf sleeve; bearing clearance is not modeled as a cavity.",
    )
    ctx.allow_overlap(
        support_frame,
        bridge_leaf,
        elem_a=support_pin_right,
        elem_b=sleeve_right,
        reason="Right hinge pin nests inside the leaf sleeve; bearing clearance is not modeled as a cavity.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = leaf_hinge.motion_limits
    ctx.check(
        "hinge_axis_is_transverse",
        leaf_hinge.axis == (0.0, -1.0, 0.0),
        details=f"Expected hinge axis (0.0, -1.0, 0.0), got {leaf_hinge.axis!r}.",
    )
    ctx.check(
        "drawbridge_opening_limit_is_realistic",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.10 <= limits.upper <= 1.30,
        details=(
            "Expected a practical single-leaf opening range from fully closed to "
            f"roughly 65-75 degrees; got {limits!r}."
        ),
    )

    ctx.expect_contact(
        bridge_leaf,
        support_frame,
        elem_a=leaf_girder_left,
        elem_b=support_seat_left,
    )
    ctx.expect_contact(
        bridge_leaf,
        support_frame,
        elem_a=leaf_girder_right,
        elem_b=support_seat_right,
    )
    ctx.expect_contact(
        bridge_leaf,
        support_frame,
        elem_a=sleeve_left,
        elem_b=tower_left_web,
    )
    ctx.expect_contact(
        bridge_leaf,
        support_frame,
        elem_a=sleeve_right,
        elem_b=tower_right_web,
    )
    ctx.expect_overlap(
        bridge_leaf,
        support_frame,
        axes="xz",
        min_overlap=0.038,
        elem_a=sleeve_left,
        elem_b=support_pin_left,
    )
    ctx.expect_overlap(
        bridge_leaf,
        support_frame,
        axes="xz",
        min_overlap=0.038,
        elem_a=sleeve_right,
        elem_b=support_pin_right,
    )
    ctx.expect_overlap(
        bridge_leaf,
        support_frame,
        axes="x",
        min_overlap=0.40,
        elem_a=root_deck_plate,
        elem_b=saddle_plate,
    )
    ctx.expect_contact(
        bridge_leaf,
        support_frame,
        elem_a=sleeve_left,
        elem_b=tower_left_web,
    )
    ctx.expect_contact(
        bridge_leaf,
        support_frame,
        elem_a=sleeve_right,
        elem_b=tower_right_web,
    )
    ctx.expect_gap(
        bridge_leaf,
        support_frame,
        axis="z",
        min_gap=0.55,
        max_gap=0.75,
        positive_elem=root_deck_plate,
        negative_elem=saddle_plate,
    )
    ctx.expect_overlap(
        bridge_leaf,
        support_frame,
        axes="yz",
        min_overlap=0.05,
        elem_a=sleeve_left,
        elem_b=housing_left,
    )
    ctx.expect_overlap(
        bridge_leaf,
        support_frame,
        axes="yz",
        min_overlap=0.05,
        elem_a=sleeve_right,
        elem_b=housing_right,
    )

    support_aabb = ctx.part_world_aabb(support_frame)
    leaf_aabb = ctx.part_world_aabb(bridge_leaf)
    rest_nose_aabb = ctx.part_element_world_aabb(bridge_leaf, elem=nose_beam)
    if support_aabb is None or leaf_aabb is None or rest_nose_aabb is None:
        ctx.fail(
            "measurable_rest_pose",
            "Support frame or bridge leaf bounds were unavailable.",
        )
        return ctx.report()

    rest_deck_aabb = ctx.part_element_world_aabb(bridge_leaf, elem=deck_plate)
    if rest_deck_aabb is None:
        ctx.fail("measurable_leaf_deck", "Bridge leaf deck bounds were unavailable.")
        return ctx.report()

    leaf_length = leaf_aabb[1][0] - leaf_aabb[0][0]
    leaf_width = leaf_aabb[1][1] - leaf_aabb[0][1]
    support_width = support_aabb[1][1] - support_aabb[0][1]
    deck_width = rest_deck_aabb[1][1] - rest_deck_aabb[0][1]

    ctx.check(
        "drawbridge_has_realistic_utility_scale",
        leaf_length >= 8.0 and leaf_width >= 3.5 and support_width >= 4.0,
        details=(
            f"Expected a full-scale utility bridge. Got leaf length={leaf_length:.3f} m, "
            f"leaf width={leaf_width:.3f} m, support width={support_width:.3f} m."
        ),
    )
    ctx.check(
        "deck_reads_as_vehicle_or_service_lane_width",
        deck_width >= 3.0,
        details=f"Deck width={deck_width:.3f} m was too narrow for a rugged utility leaf.",
    )
    ctx.check(
        "leaf_projects_well_beyond_abutment",
        rest_deck_aabb[1][0] > support_aabb[1][0] + 5.80,
        details=(
            f"Leaf deck max x={rest_deck_aabb[1][0]:.3f} should extend at least 5.80 m "
            f"beyond support max x={support_aabb[1][0]:.3f}."
        ),
    )
    ctx.check(
        "nose_beam_reads_as_full_width_leaf_tip",
        (rest_nose_aabb[1][1] - rest_nose_aabb[0][1]) >= 3.40,
        details=(
            f"Nose beam width={(rest_nose_aabb[1][1] - rest_nose_aabb[0][1]):.3f} "
            "was too narrow to read as a robust bridge tip."
        ),
    )

    open_nose_aabb = None
    with ctx.pose({leaf_hinge: 1.0}):
        ctx.expect_overlap(
            bridge_leaf,
            support_frame,
            axes="xz",
            min_overlap=0.038,
            elem_a=sleeve_left,
            elem_b=support_pin_left,
        )
        ctx.expect_overlap(
            bridge_leaf,
            support_frame,
            axes="xz",
            min_overlap=0.038,
            elem_a=sleeve_right,
            elem_b=support_pin_right,
        )
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            min_gap=0.40,
            positive_elem=leaf_girder_left,
            negative_elem=support_seat_left,
        )
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            min_gap=0.40,
            positive_elem=leaf_girder_right,
            negative_elem=support_seat_right,
        )
        open_nose_aabb = ctx.part_element_world_aabb(bridge_leaf, elem=nose_beam)

    if open_nose_aabb is None:
        ctx.fail("measurable_open_pose", "Open-pose nose beam bounds were unavailable.")
        return ctx.report()

    ctx.check(
        "bridge_leaf_lifts_clear_when_opened",
        open_nose_aabb[0][2] > rest_nose_aabb[1][2] + 5.50,
        details=(
            f"Open nose beam min z={open_nose_aabb[0][2]:.3f} did not rise enough above "
            f"rest nose beam max z={rest_nose_aabb[1][2]:.3f}."
        ),
    )
    ctx.check(
        "bridge_leaf_swings_back_toward_hinge_when_opened",
        open_nose_aabb[1][0] < rest_nose_aabb[1][0] - 3.30,
        details=(
            f"Open nose beam max x={open_nose_aabb[1][0]:.3f} should retract well behind "
            f"rest max x={rest_nose_aabb[1][0]:.3f}."
        ),
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({leaf_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="bridge_leaf_lower_no_overlap"
            )
            ctx.fail_if_isolated_parts(name="bridge_leaf_lower_no_floating")
        with ctx.pose({leaf_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="bridge_leaf_upper_no_overlap"
            )
            ctx.fail_if_isolated_parts(name="bridge_leaf_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
