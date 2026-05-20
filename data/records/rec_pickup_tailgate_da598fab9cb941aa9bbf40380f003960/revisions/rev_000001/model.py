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
    rounded_rect_profile,
)


def _rounded_strap_mesh(length: float, width: float, thickness: float, name: str):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, length, radius=width * 0.48, corner_segments=8),
        thickness,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_rigid_stays")

    gate_width = 1.52
    gate_height = 0.54
    gate_thickness = 0.06
    hinge_z = 0.08

    side_gap = 0.05
    stub_width = 0.10
    stub_depth = 0.18
    stub_height = 0.34
    overall_width = gate_width + 2.0 * (side_gap + stub_width)

    stub_rear_y = 0.05
    support_x = gate_width * 0.5 + 0.03
    upper_pivot_y = 0.012
    upper_pivot_z = hinge_z + 0.005
    gate_stay_pivot_y = 0.012
    gate_stay_pivot_z = 0.20
    stay_bar_length = 0.18
    stay_tip_length = 0.035
    closed_stay_upper_rpy = math.radians(9.7)
    closed_stay_lower_rpy = math.radians(-69.6)

    body_paint = model.material("body_paint", rgba=(0.18, 0.29, 0.43, 1.0))
    inner_panel = model.material("inner_panel", rgba=(0.16, 0.18, 0.20, 1.0))
    hardware = model.material("hardware", rgba=(0.28, 0.30, 0.33, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.12, 0.12, 0.12, 1.0))

    stay_mesh = _rounded_strap_mesh(
        stay_bar_length,
        width=0.026,
        thickness=0.008,
        name="support_stay_blade",
    )

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((overall_width - 0.08, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.17, 0.03)),
        material=inner_panel,
        name="bed_bridge",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        stub_x = side_sign * (gate_width / 2.0 + side_gap + stub_width / 2.0)
        bed_frame.visual(
            Box((stub_width, stub_depth, stub_height)),
            origin=Origin(xyz=(stub_x, stub_rear_y + stub_depth / 2.0, stub_height / 2.0)),
            material=body_paint,
            name=f"{side_name}_stub",
        )
        bed_frame.visual(
            Box((stub_width * 0.95, 0.10, 0.035)),
            origin=Origin(xyz=(stub_x, 0.10, stub_height - 0.0175)),
            material=body_paint,
            name=f"{side_name}_stub_cap",
        )
        bed_frame.visual(
            Box((0.028, 0.040, 0.045)),
            origin=Origin(
                xyz=(side_sign * (support_x + 0.014), 0.050, upper_pivot_z),
            ),
            material=hinge_black,
            name=f"{side_name}_upper_arm",
        )
        bed_frame.visual(
            Box((0.024, 0.020, 0.030)),
            origin=Origin(
                xyz=(side_sign * support_x, 0.028, upper_pivot_z),
            ),
            material=hinge_black,
            name=f"{side_name}_upper_bracket",
        )
        bed_frame.visual(
            Box((0.042, 0.028, 0.050)),
            origin=Origin(
                xyz=(side_sign * (gate_width / 2.0 + 0.028), 0.042, hinge_z),
            ),
            material=hinge_black,
            name=f"{side_name}_hinge_box",
        )
    bed_frame.inertial = Inertial.from_geometry(
        Box((overall_width, 0.23, stub_height)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.115, stub_height / 2.0)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((gate_width, 0.032, 0.28)),
        origin=Origin(xyz=(0.0, -0.016, 0.14)),
        material=body_paint,
        name="outer_skin_lower",
    )
    tailgate.visual(
        Box((0.63, 0.032, 0.10)),
        origin=Origin(xyz=(-0.445, -0.016, 0.39)),
        material=body_paint,
        name="outer_skin_left",
    )
    tailgate.visual(
        Box((0.63, 0.032, 0.10)),
        origin=Origin(xyz=(0.445, -0.016, 0.39)),
        material=body_paint,
        name="outer_skin_right",
    )
    tailgate.visual(
        Box((gate_width, 0.032, 0.10)),
        origin=Origin(xyz=(0.0, -0.016, 0.49)),
        material=body_paint,
        name="outer_skin_upper",
    )
    tailgate.visual(
        Box((gate_width, 0.050, 0.03)),
        origin=Origin(xyz=(0.0, -0.015, gate_height - 0.015)),
        material=body_paint,
        name="top_cap",
    )
    tailgate.visual(
        Box((gate_width, 0.050, 0.075)),
        origin=Origin(xyz=(0.0, -0.015, 0.0375)),
        material=body_paint,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((0.032, 0.050, gate_height - 0.105)),
        origin=Origin(
            xyz=(-gate_width / 2.0 + 0.016, -0.015, 0.255),
        ),
        material=body_paint,
        name="left_side_hem",
    )
    tailgate.visual(
        Box((0.032, 0.050, gate_height - 0.105)),
        origin=Origin(
            xyz=(gate_width / 2.0 - 0.016, -0.015, 0.255),
        ),
        material=body_paint,
        name="right_side_hem",
    )
    tailgate.visual(
        Box((gate_width - 0.064, 0.024, 0.48)),
        origin=Origin(xyz=(0.0, 0.008, 0.27)),
        material=inner_panel,
        name="inner_panel",
    )
    tailgate.visual(
        Box((0.24, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, -0.014, 0.405)),
        material=inner_panel,
        name="handle_bezel",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        tailgate.visual(
            Box((0.046, 0.018, 0.080)),
            origin=Origin(
                xyz=(side_sign * (gate_width / 2.0 - 0.018), 0.011, gate_stay_pivot_z),
            ),
            material=hinge_black,
            name=f"{side_name}_stay_brace",
        )
        tailgate.visual(
            Box((0.016, 0.014, 0.050)),
            origin=Origin(
                xyz=(side_sign * (gate_width / 2.0 + 0.008), 0.023, gate_stay_pivot_z),
            ),
            material=hinge_black,
            name=f"{side_name}_stay_link",
        )
        tailgate.visual(
            Box((0.028, 0.018, 0.050)),
            origin=Origin(
                xyz=(side_sign * support_x, 0.028, gate_stay_pivot_z),
            ),
            material=hinge_black,
            name=f"{side_name}_stay_bracket",
        )
        tailgate.visual(
            Box((0.040, 0.028, 0.052)),
            origin=Origin(
                xyz=(side_sign * (gate_width / 2.0 - 0.012), 0.014, 0.038),
            ),
            material=hinge_black,
            name=f"{side_name}_hinge_ear",
        )
    tailgate.inertial = Inertial.from_geometry(
        Box((gate_width, 0.08, gate_height)),
        mass=17.0,
        origin=Origin(xyz=(0.0, -0.018, gate_height / 2.0)),
    )

    left_stay = model.part("left_stay")
    left_stay.visual(
        Box((0.014, 0.008, 0.164)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=hardware,
        name="blade",
    )
    left_stay.visual(
        Box((0.018, 0.020, 0.018)),
        origin=Origin(),
        material=hinge_black,
        name="upper_pad",
    )
    left_stay.visual(
        Box((0.014, 0.004, 0.020)),
        origin=Origin(
            xyz=(0.0, -0.006, 0.170),
        ),
        material=hinge_black,
        name="lower_clevis",
    )
    left_stay.visual(
        Box((0.014, 0.004, 0.020)),
        origin=Origin(
            xyz=(0.0, 0.006, 0.170),
        ),
        material=hinge_black,
        name="outer_clevis",
    )
    left_stay.inertial = Inertial.from_geometry(
        Box((0.026, 0.026, stay_bar_length)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, stay_bar_length / 2.0)),
    )

    right_stay = model.part("right_stay")
    right_stay.visual(
        Box((0.014, 0.008, 0.164)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=hardware,
        name="blade",
    )
    right_stay.visual(
        Box((0.018, 0.020, 0.018)),
        origin=Origin(),
        material=hinge_black,
        name="upper_pad",
    )
    right_stay.visual(
        Box((0.014, 0.004, 0.020)),
        origin=Origin(
            xyz=(0.0, -0.006, 0.170),
        ),
        material=hinge_black,
        name="lower_clevis",
    )
    right_stay.visual(
        Box((0.014, 0.004, 0.020)),
        origin=Origin(
            xyz=(0.0, 0.006, 0.170),
        ),
        material=hinge_black,
        name="outer_clevis",
    )
    right_stay.inertial = Inertial.from_geometry(
        Box((0.026, 0.026, stay_bar_length)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, stay_bar_length / 2.0)),
    )

    left_stay_tip = model.part("left_stay_tip")
    left_stay_tip.visual(
        Box((0.012, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=hardware,
        name="tip_link",
    )
    left_stay_tip.visual(
        Box((0.012, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=hinge_black,
        name="inner_pad",
    )
    left_stay_tip.visual(
        Box((0.016, 0.004, 0.010)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.030),
        ),
        material=hinge_black,
        name="gate_pin",
    )
    left_stay_tip.inertial = Inertial.from_geometry(
        Box((0.018, 0.020, stay_tip_length)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, stay_tip_length / 2.0)),
    )

    right_stay_tip = model.part("right_stay_tip")
    right_stay_tip.visual(
        Box((0.012, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=hardware,
        name="tip_link",
    )
    right_stay_tip.visual(
        Box((0.012, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=hinge_black,
        name="inner_pad",
    )
    right_stay_tip.visual(
        Box((0.016, 0.004, 0.010)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.030),
        ),
        material=hinge_black,
        name="gate_pin",
    )
    right_stay_tip.inertial = Inertial.from_geometry(
        Box((0.018, 0.020, stay_tip_length)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, stay_tip_length / 2.0)),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_black,
        name="handle_barrel",
    )
    latch_handle.visual(
        Box((0.150, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, -0.014, -0.018)),
        material=handle_black,
        name="handle_paddle",
    )
    latch_handle.visual(
        Box((0.055, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.026, -0.018)),
        material=handle_black,
        name="handle_grip",
    )
    latch_handle.inertial = Inertial.from_geometry(
        Box((0.180, 0.040, 0.060)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.015, -0.018)),
    )

    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "left_stay_upper",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=left_stay,
        origin=Origin(xyz=(-support_x, upper_pivot_y, upper_pivot_z), rpy=(closed_stay_upper_rpy, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=math.radians(-15.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "left_stay_lower",
        ArticulationType.REVOLUTE,
        parent=left_stay,
        child=left_stay_tip,
        origin=Origin(xyz=(0.0, 0.0, stay_bar_length), rpy=(closed_stay_lower_rpy, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=math.radians(-20.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "right_stay_upper",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=right_stay,
        origin=Origin(xyz=(support_x, upper_pivot_y, upper_pivot_z), rpy=(closed_stay_upper_rpy, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=math.radians(-15.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "right_stay_lower",
        ArticulationType.REVOLUTE,
        parent=right_stay,
        child=right_stay_tip,
        origin=Origin(xyz=(0.0, 0.0, stay_bar_length), rpy=(closed_stay_lower_rpy, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=math.radians(-20.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_handle,
        origin=Origin(xyz=(0.0, -0.032, 0.405)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=math.radians(-35.0),
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    left_stay = object_model.get_part("left_stay")
    right_stay = object_model.get_part("right_stay")
    left_stay_tip = object_model.get_part("left_stay_tip")
    right_stay_tip = object_model.get_part("right_stay_tip")
    latch_handle = object_model.get_part("latch_handle")

    gate_hinge = object_model.get_articulation("gate_hinge")
    left_stay_upper = object_model.get_articulation("left_stay_upper")
    left_stay_lower = object_model.get_articulation("left_stay_lower")
    right_stay_upper = object_model.get_articulation("right_stay_upper")
    right_stay_lower = object_model.get_articulation("right_stay_lower")
    handle_hinge = object_model.get_articulation("handle_hinge")

    def _elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    def _check_elem_proximity(
        name: str,
        part_a,
        elem_a: str,
        part_b,
        elem_b: str,
        *,
        max_distance: float,
        max_x_delta: float = 0.003,
    ):
        center_a = _elem_center(part_a, elem_a)
        center_b = _elem_center(part_b, elem_b)
        ok = (
            center_a is not None
            and center_b is not None
            and abs(center_a[0] - center_b[0]) <= max_x_delta
            and math.dist(center_a, center_b) <= max_distance
        )
        details = (
            f"{elem_a}={center_a} {elem_b}={center_b} "
            f"max_distance={max_distance} max_x_delta={max_x_delta}"
        )
        ctx.check(name, ok, details=details)

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        bed_frame,
        left_stay,
        elem_a="left_upper_bracket",
        elem_b="upper_pad",
        reason="The left bedside pivot is represented as a boxed clevis enclosing the stay's pivot pad.",
    )
    ctx.allow_overlap(
        bed_frame,
        right_stay,
        elem_a="right_upper_bracket",
        elem_b="upper_pad",
        reason="The right bedside pivot is represented as a boxed clevis enclosing the stay's pivot pad.",
    )
    ctx.allow_overlap(
        bed_frame,
        left_stay,
        elem_a="left_upper_bracket",
        elem_b="blade",
        reason="The left stay strap is simplified as a flush blade passing through a boxed pivot capture.",
    )
    ctx.allow_overlap(
        bed_frame,
        right_stay,
        elem_a="right_upper_bracket",
        elem_b="blade",
        reason="The right stay strap is simplified as a flush blade passing through a boxed pivot capture.",
    )
    ctx.allow_overlap(
        left_stay,
        left_stay_tip,
        elem_a="lower_clevis",
        elem_b="inner_pad",
        reason="The left gate-side stay joint is represented as nested clevis hardware around the tip link.",
    )
    ctx.allow_overlap(
        right_stay,
        right_stay_tip,
        elem_a="lower_clevis",
        elem_b="inner_pad",
        reason="The right gate-side stay joint is represented as nested clevis hardware around the tip link.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    expected_axis = (1.0, 0.0, 0.0)
    ctx.check(
        "all articulated axes run across the truck",
        all(
            joint.axis == expected_axis
            for joint in (
                gate_hinge,
                left_stay_upper,
                left_stay_lower,
                right_stay_upper,
                right_stay_lower,
                handle_hinge,
            )
        ),
        details="Tailgate hinge, both stay pivots, and the handle should all rotate about local X.",
    )
    gate_limits = gate_hinge.motion_limits
    ctx.check(
        "tailgate has realistic lowering travel",
        gate_limits is not None
        and gate_limits.lower == 0.0
        and gate_limits.upper is not None
        and gate_limits.upper >= math.radians(85.0),
        details="The tailgate should rotate downward from closed toward a near-horizontal open position.",
    )

    ctx.expect_overlap(
        tailgate,
        bed_frame,
        axes="x",
        min_overlap=1.48,
        name="tailgate spans between bedside stubs",
    )
    ctx.expect_gap(
        bed_frame,
        tailgate,
        axis="y",
        positive_elem="left_hinge_box",
        negative_elem="left_hinge_ear",
        max_gap=0.001,
        max_penetration=0.0,
        name="left hinge ear closes flush to the bedside hinge box",
    )
    ctx.expect_gap(
        bed_frame,
        tailgate,
        axis="y",
        positive_elem="right_hinge_box",
        negative_elem="right_hinge_ear",
        max_gap=0.001,
        max_penetration=0.0,
        name="right hinge ear closes flush to the bedside hinge box",
    )
    _check_elem_proximity(
        "left upper stay pivot remains aligned in the bedside bracket",
        bed_frame,
        "left_upper_bracket",
        left_stay,
        "upper_pad",
        max_distance=0.022,
    )
    _check_elem_proximity(
        "right upper stay pivot remains aligned in the bedside bracket",
        bed_frame,
        "right_upper_bracket",
        right_stay,
        "upper_pad",
        max_distance=0.022,
    )
    _check_elem_proximity(
        "left lower stay pivot remains aligned with the tailgate bracket",
        tailgate,
        "left_stay_bracket",
        left_stay_tip,
        "gate_pin",
        max_distance=0.022,
    )
    _check_elem_proximity(
        "right lower stay pivot remains aligned with the tailgate bracket",
        tailgate,
        "right_stay_bracket",
        right_stay_tip,
        "gate_pin",
        max_distance=0.022,
    )
    ctx.expect_contact(
        latch_handle,
        tailgate,
        elem_a="handle_barrel",
        elem_b="handle_bezel",
        name="handle sits on its bezel pivot pocket",
    )

    open_pose = {
        gate_hinge: math.radians(86.0),
        left_stay_upper: math.radians(79.0),
        right_stay_upper: math.radians(79.0),
        left_stay_lower: math.radians(42.3),
        right_stay_lower: math.radians(42.3),
    }
    with ctx.pose(open_pose):
        _check_elem_proximity(
            "left upper stay pivot stays aligned when gate is lowered",
            bed_frame,
            "left_upper_bracket",
            left_stay,
            "upper_pad",
            max_distance=0.022,
        )
        _check_elem_proximity(
            "right upper stay pivot stays aligned when gate is lowered",
            bed_frame,
            "right_upper_bracket",
            right_stay,
            "upper_pad",
            max_distance=0.022,
        )
        _check_elem_proximity(
            "left lower stay pivot stays aligned when gate is lowered",
            tailgate,
            "left_stay_bracket",
            left_stay_tip,
            "gate_pin",
            max_distance=0.024,
        )
        _check_elem_proximity(
            "right lower stay pivot stays aligned when gate is lowered",
            tailgate,
            "right_stay_bracket",
            right_stay_tip,
            "gate_pin",
            max_distance=0.024,
        )
        open_gate_aabb = ctx.part_world_aabb(tailgate)
        ctx.check(
            "lowered tailgate reads nearly horizontal",
            open_gate_aabb is not None
            and (open_gate_aabb[1][2] - open_gate_aabb[0][2]) < 0.12
            and open_gate_aabb[0][1] < -0.40,
            details="In the lowered pose the tailgate should stretch rearward and collapse to a shallow vertical thickness.",
        )

    closed_handle_aabb = ctx.part_element_world_aabb(latch_handle, elem="handle_paddle")
    with ctx.pose({handle_hinge: math.radians(-28.0)}):
        open_handle_aabb = ctx.part_element_world_aabb(latch_handle, elem="handle_paddle")
        ctx.check(
            "handle rotates outward on its local pivot",
            closed_handle_aabb is not None
            and open_handle_aabb is not None
            and open_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.010,
            details="Opening the latch should move the handle farther outward from the tailgate skin.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
