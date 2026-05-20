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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_sliding_security_gate")

    galvanized = model.material("galvanized", rgba=(0.67, 0.70, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.93, 0.78, 0.14, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.78, 0.18, 0.15, 1.0))
    guide_white = model.material("guide_white", rgba=(0.88, 0.89, 0.84, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    def y_cylinder_origin(x: float, y: float, z: float) -> Origin:
        return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))

    def add_y_bolt(
        part,
        *,
        x: float,
        y: float,
        z: float,
        length: float,
        radius: float,
        material,
        name: str,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=y_cylinder_origin(x, y, z),
            material=material,
            name=name,
        )

    def add_anchor_bolts(part, *, center_x: float, prefix: str) -> None:
        for idx, (dx, dy) in enumerate(((-0.06, -0.08), (-0.06, 0.08), (0.06, -0.08), (0.06, 0.08)), start=1):
            part.visual(
                Cylinder(radius=0.008, length=0.04),
                origin=Origin(xyz=(center_x + dx, dy, 0.04)),
                material=dark_steel,
                name=f"{prefix}_anchor_{idx}",
            )

    fixed_frame = model.part("fixed_frame")

    left_post_x = 0.06
    receiver_post_x = 1.78
    parking_post_x = 3.32
    span_center_x = 1.69
    track_center_x = 1.69
    slide_origin = (0.18, 0.0, 0.29)
    slide_travel = 1.54

    fixed_frame.visual(
        Box((0.12, 0.18, 2.22)),
        origin=Origin(xyz=(left_post_x, 0.0, 1.11)),
        material=galvanized,
        name="left_support_post",
    )
    fixed_frame.visual(
        Box((0.12, 0.18, 2.22)),
        origin=Origin(xyz=(receiver_post_x, 0.0, 1.11)),
        material=galvanized,
        name="receiver_post",
    )
    fixed_frame.visual(
        Box((0.12, 0.18, 2.22)),
        origin=Origin(xyz=(parking_post_x, 0.0, 1.11)),
        material=galvanized,
        name="parking_post",
    )

    for center_x, prefix in (
        (left_post_x, "left_base"),
        (receiver_post_x, "receiver_base"),
        (parking_post_x, "parking_base"),
    ):
        fixed_frame.visual(
            Box((0.24, 0.28, 0.02)),
            origin=Origin(xyz=(center_x, 0.0, 0.01)),
            material=dark_steel,
            name=f"{prefix}_plate",
        )
        add_anchor_bolts(fixed_frame, center_x=center_x, prefix=prefix)

    fixed_frame.visual(
        Box((3.38, 0.16, 0.18)),
        origin=Origin(xyz=(span_center_x, 0.0, 0.09)),
        material=galvanized,
        name="bottom_sill",
    )
    fixed_frame.visual(
        Box((3.38, 0.18, 0.06)),
        origin=Origin(xyz=(span_center_x, 0.0, 2.19)),
        material=galvanized,
        name="header_beam",
    )

    fixed_frame.visual(
        Box((3.14, 0.07, 0.01)),
        origin=Origin(xyz=(track_center_x, 0.0, 0.185)),
        material=dark_steel,
        name="guide_floor",
    )
    fixed_frame.visual(
        Box((3.14, 0.02, 0.10)),
        origin=Origin(xyz=(track_center_x, 0.045, 0.24)),
        material=dark_steel,
        name="guide_front_rail",
    )
    fixed_frame.visual(
        Box((3.14, 0.02, 0.10)),
        origin=Origin(xyz=(track_center_x, -0.045, 0.24)),
        material=dark_steel,
        name="guide_rear_rail",
    )

    fixed_frame.visual(
        Box((3.14, 0.18, 0.02)),
        origin=Origin(xyz=(track_center_x, 0.0, 2.15)),
        material=dark_steel,
        name="track_top_plate",
    )
    fixed_frame.visual(
        Box((3.14, 0.018, 0.08)),
        origin=Origin(xyz=(track_center_x, 0.081, 2.10)),
        material=dark_steel,
        name="track_front_wall",
    )
    fixed_frame.visual(
        Box((3.14, 0.018, 0.08)),
        origin=Origin(xyz=(track_center_x, -0.081, 2.10)),
        material=dark_steel,
        name="track_rear_wall",
    )
    fixed_frame.visual(
        Box((3.14, 0.05, 0.02)),
        origin=Origin(xyz=(track_center_x, 0.055, 2.05)),
        material=dark_steel,
        name="track_front_flange",
    )
    fixed_frame.visual(
        Box((3.14, 0.05, 0.02)),
        origin=Origin(xyz=(track_center_x, -0.055, 2.05)),
        material=dark_steel,
        name="track_rear_flange",
    )

    brace_angle = math.atan2(0.26, 0.26)
    fixed_frame.visual(
        Box((0.37, 0.02, 0.08)),
        origin=Origin(xyz=(0.20, 0.07, 1.96), rpy=(0.0, -brace_angle, 0.0)),
        material=safety_yellow,
        name="left_upper_brace_front",
    )
    fixed_frame.visual(
        Box((0.37, 0.02, 0.08)),
        origin=Origin(xyz=(0.20, -0.07, 1.96), rpy=(0.0, -brace_angle, 0.0)),
        material=safety_yellow,
        name="left_upper_brace_rear",
    )
    fixed_frame.visual(
        Box((0.37, 0.02, 0.08)),
        origin=Origin(xyz=(3.18, 0.07, 1.96), rpy=(0.0, brace_angle, 0.0)),
        material=safety_yellow,
        name="parking_upper_brace_front",
    )
    fixed_frame.visual(
        Box((0.37, 0.02, 0.08)),
        origin=Origin(xyz=(3.18, -0.07, 1.96), rpy=(0.0, brace_angle, 0.0)),
        material=safety_yellow,
        name="parking_upper_brace_rear",
    )

    fixed_frame.visual(
        Box((0.04, 0.08, 0.10)),
        origin=Origin(xyz=(0.28, 0.0, 2.10)),
        material=safety_yellow,
        name="closed_top_stop",
    )
    fixed_frame.visual(
        Box((0.04, 0.08, 0.10)),
        origin=Origin(xyz=(3.08, 0.0, 2.10)),
        material=safety_yellow,
        name="open_top_stop",
    )
    fixed_frame.visual(
        Box((0.04, 0.09, 0.10)),
        origin=Origin(xyz=(0.24, 0.0, 0.24)),
        material=safety_yellow,
        name="closed_bottom_stop",
    )
    fixed_frame.visual(
        Box((0.04, 0.09, 0.10)),
        origin=Origin(xyz=(3.20, 0.0, 0.24)),
        material=safety_yellow,
        name="open_bottom_stop",
    )
    add_y_bolt(
        fixed_frame,
        x=0.28,
        y=0.0,
        z=2.10,
        length=0.10,
        radius=0.007,
        material=dark_steel,
        name="closed_top_stop_bolt",
    )
    add_y_bolt(
        fixed_frame,
        x=3.08,
        y=0.0,
        z=2.10,
        length=0.10,
        radius=0.007,
        material=dark_steel,
        name="open_top_stop_bolt",
    )

    fixed_frame.visual(
        Box((0.06, 0.04, 0.24)),
        origin=Origin(xyz=(1.75, 0.055, 0.99)),
        material=galvanized,
        name="keeper_block",
    )
    fixed_frame.visual(
        Box((0.12, 0.012, 0.42)),
        origin=Origin(xyz=(1.74, 0.096, 1.26)),
        material=safety_yellow,
        name="receiver_guard_backer",
    )
    fixed_frame.visual(
        Box((0.12, 0.014, 0.18)),
        origin=Origin(xyz=(1.74, 0.111, 1.32)),
        material=safety_yellow,
        name="lockout_shroud_front",
    )
    fixed_frame.visual(
        Box((0.12, 0.004, 0.10)),
        origin=Origin(xyz=(1.74, 0.103, 1.32)),
        material=safety_yellow,
        name="lockout_shroud_bridge",
    )
    fixed_frame.visual(
        Box((0.12, 0.014, 0.18)),
        origin=Origin(xyz=(1.74, 0.085, 1.32)),
        material=safety_yellow,
        name="lockout_shroud_rear",
    )
    fixed_frame.visual(
        Box((0.08, 0.05, 0.10)),
        origin=Origin(xyz=(1.74, 0.095, 1.47)),
        material=safety_yellow,
        name="receiver_guard_cap",
    )
    add_y_bolt(
        fixed_frame,
        x=1.75,
        y=0.055,
        z=0.99,
        length=0.08,
        radius=0.008,
        material=dark_steel,
        name="keeper_bolt",
    )

    fixed_frame.inertial = Inertial.from_geometry(
        Box((3.38, 0.28, 2.22)),
        mass=280.0,
        origin=Origin(xyz=(span_center_x, 0.0, 1.11)),
    )

    gate_leaf = model.part("gate_leaf")

    gate_leaf.visual(
        Box((0.06, 0.05, 1.72)),
        origin=Origin(xyz=(0.03, 0.0, 0.86)),
        material=galvanized,
        name="rear_stile",
    )
    gate_leaf.visual(
        Box((0.06, 0.05, 1.72)),
        origin=Origin(xyz=(1.51, 0.0, 0.86)),
        material=galvanized,
        name="leading_stile",
    )
    gate_leaf.visual(
        Box((1.54, 0.05, 0.08)),
        origin=Origin(xyz=(0.77, 0.0, 0.04)),
        material=galvanized,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((1.54, 0.05, 0.06)),
        origin=Origin(xyz=(0.77, 0.0, 1.69)),
        material=galvanized,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((1.42, 0.04, 0.05)),
        origin=Origin(xyz=(0.77, 0.0, 0.95)),
        material=galvanized,
        name="mid_rail",
    )

    for idx, x_pos in enumerate((0.18, 0.37, 0.56, 0.75, 0.94, 1.13, 1.32), start=1):
        gate_leaf.visual(
            Box((0.03, 0.02, 1.55)),
            origin=Origin(xyz=(x_pos, 0.0, 0.855)),
            material=dark_steel,
            name=f"infill_bar_{idx}",
        )

    gate_leaf.visual(
        Box((1.86, 0.02, 0.06)),
        origin=Origin(xyz=(0.76, -0.018, 0.88), rpy=(0.0, -math.atan2(1.48, 1.54), 0.0)),
        material=dark_steel,
        name="diagonal_brace",
    )

    for prefix, x_pos in (("rear", 0.36), ("front", 1.18)):
        gate_leaf.visual(
            Box((0.18, 0.012, 0.12)),
            origin=Origin(xyz=(x_pos, 0.0, 1.66)),
            material=safety_yellow,
            name=f"{prefix}_hanger_doubler",
        )
        gate_leaf.visual(
            Box((0.04, 0.012, 0.16)),
            origin=Origin(xyz=(x_pos, 0.0, 1.76)),
            material=galvanized,
            name=f"{prefix}_hanger_plate",
        )
        gate_leaf.visual(
            Box((0.16, 0.03, 0.02)),
            origin=Origin(xyz=(x_pos, 0.0, 1.83)),
            material=safety_yellow,
            name=f"{prefix}_roller_guard",
        )
        gate_leaf.visual(
            Box((0.16, 0.02, 0.08)),
            origin=Origin(xyz=(x_pos - 0.05, 0.0, 1.74), rpy=(0.0, brace_angle, 0.0)),
            material=galvanized,
            name=f"{prefix}_rear_gusset",
        )
        gate_leaf.visual(
            Box((0.16, 0.02, 0.08)),
            origin=Origin(xyz=(x_pos + 0.05, 0.0, 1.74), rpy=(0.0, -brace_angle, 0.0)),
            material=galvanized,
            name=f"{prefix}_front_gusset",
        )
        add_y_bolt(
            gate_leaf,
            x=x_pos,
            y=0.0,
            z=1.63,
            length=0.06,
            radius=0.007,
            material=dark_steel,
            name=f"{prefix}_hanger_lower_bolt",
        )
        add_y_bolt(
            gate_leaf,
            x=x_pos,
            y=0.0,
            z=1.70,
            length=0.06,
            radius=0.007,
            material=dark_steel,
            name=f"{prefix}_hanger_upper_bolt",
        )
        add_y_bolt(
            gate_leaf,
            x=x_pos,
            y=0.0,
            z=1.80,
            length=0.13,
            radius=0.006,
            material=dark_steel,
            name=f"{prefix}_trolley_axle",
        )
        gate_leaf.visual(
            Cylinder(radius=0.03, length=0.018),
            origin=y_cylinder_origin(x_pos, 0.055, 1.80),
            material=rubber,
            name=f"{prefix}_trolley_front_wheel",
        )
        gate_leaf.visual(
            Cylinder(radius=0.03, length=0.018),
            origin=y_cylinder_origin(x_pos, -0.055, 1.80),
            material=rubber,
            name=f"{prefix}_trolley_rear_wheel",
        )

    gate_leaf.visual(
        Box((0.04, 0.04, 0.08)),
        origin=Origin(xyz=(0.14, 0.0, 1.80)),
        material=rubber,
        name="closed_bumper",
    )
    gate_leaf.visual(
        Box((0.04, 0.02, 0.14)),
        origin=Origin(xyz=(0.14, 0.0, 1.76)),
        material=galvanized,
        name="closed_bumper_stem",
    )
    gate_leaf.visual(
        Box((0.04, 0.04, 0.08)),
        origin=Origin(xyz=(1.32, 0.0, 1.80)),
        material=rubber,
        name="open_bumper",
    )
    gate_leaf.visual(
        Box((0.04, 0.02, 0.14)),
        origin=Origin(xyz=(1.32, 0.0, 1.76)),
        material=galvanized,
        name="open_bumper_stem",
    )

    gate_leaf.visual(
        Box((0.06, 0.034, 0.10)),
        origin=Origin(xyz=(0.30, 0.0, -0.05)),
        material=guide_white,
        name="guide_shoe_rear",
    )
    gate_leaf.visual(
        Box((0.06, 0.034, 0.10)),
        origin=Origin(xyz=(1.24, 0.0, -0.05)),
        material=guide_white,
        name="guide_shoe_front",
    )

    gate_leaf.visual(
        Box((0.06, 0.03, 0.22)),
        origin=Origin(xyz=(1.51, 0.040, 0.99)),
        material=galvanized,
        name="latch_block",
    )
    gate_leaf.visual(
        Box((0.04, 0.012, 0.24)),
        origin=Origin(xyz=(1.51, 0.055, 1.22)),
        material=lockout_red,
        name="lock_tab",
    )
    gate_leaf.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=y_cylinder_origin(1.51, 0.055, 1.30),
        material=lockout_red,
        name="lock_eye",
    )
    add_y_bolt(
        gate_leaf,
        x=1.50,
        y=0.040,
        z=1.08,
        length=0.05,
        radius=0.007,
        material=dark_steel,
        name="latch_bolt",
    )

    gate_leaf.inertial = Inertial.from_geometry(
        Box((1.56, 0.10, 1.86)),
        mass=120.0,
        origin=Origin(xyz=(0.78, 0.0, 0.88)),
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(xyz=slide_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.35,
            lower=0.0,
            upper=slide_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate")

    track_front_flange = fixed_frame.get_visual("track_front_flange")
    track_rear_flange = fixed_frame.get_visual("track_rear_flange")
    guide_front_rail = fixed_frame.get_visual("guide_front_rail")
    guide_rear_rail = fixed_frame.get_visual("guide_rear_rail")
    keeper_block = fixed_frame.get_visual("keeper_block")
    closed_top_stop = fixed_frame.get_visual("closed_top_stop")
    open_top_stop = fixed_frame.get_visual("open_top_stop")

    rear_front_wheel = gate_leaf.get_visual("rear_trolley_front_wheel")
    rear_rear_wheel = gate_leaf.get_visual("rear_trolley_rear_wheel")
    front_front_wheel = gate_leaf.get_visual("front_trolley_front_wheel")
    front_rear_wheel = gate_leaf.get_visual("front_trolley_rear_wheel")
    guide_shoe_rear = gate_leaf.get_visual("guide_shoe_rear")
    guide_shoe_front = gate_leaf.get_visual("guide_shoe_front")
    latch_block = gate_leaf.get_visual("latch_block")
    closed_bumper = gate_leaf.get_visual("closed_bumper")
    open_bumper = gate_leaf.get_visual("open_bumper")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    limits = slide.motion_limits
    ctx.check(
        "gate uses prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC,
        f"articulation_type={slide.articulation_type}",
    )
    ctx.check(
        "gate slide axis points along +x",
        tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        f"axis={slide.axis}",
    )
    ctx.check(
        "gate travel matches framed opening and parking bay",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - 1.54) < 1e-9,
        f"limits={limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            gate_leaf,
            fixed_frame,
            contact_tol=0.001,
            elem_a=rear_front_wheel,
            elem_b=track_front_flange,
            name="rear trolley front wheel bears on front flange when closed",
        )
        ctx.expect_contact(
            gate_leaf,
            fixed_frame,
            contact_tol=0.001,
            elem_a=rear_rear_wheel,
            elem_b=track_rear_flange,
            name="rear trolley rear wheel bears on rear flange when closed",
        )
        ctx.expect_gap(
            fixed_frame,
            gate_leaf,
            axis="y",
            positive_elem=guide_front_rail,
            negative_elem=guide_shoe_rear,
            min_gap=0.010,
            max_gap=0.020,
            name="rear guide shoe clears front rail with controlled side gap",
        )
        ctx.expect_gap(
            gate_leaf,
            fixed_frame,
            axis="y",
            positive_elem=guide_shoe_rear,
            negative_elem=guide_rear_rail,
            min_gap=0.010,
            max_gap=0.020,
            name="rear guide shoe clears rear rail with controlled side gap",
        )
        ctx.expect_gap(
            fixed_frame,
            gate_leaf,
            axis="x",
            positive_elem=keeper_block,
            negative_elem=latch_block,
            max_gap=0.001,
            max_penetration=1e-6,
            name="closed latch seats against keeper block",
        )
        ctx.expect_gap(
            gate_leaf,
            fixed_frame,
            axis="x",
            positive_elem=closed_bumper,
            negative_elem=closed_top_stop,
            max_gap=0.001,
            max_penetration=1e-6,
            name="closed over-travel stop arrests the rear bumper",
        )

    with ctx.pose({slide: 1.54}):
        ctx.expect_contact(
            gate_leaf,
            fixed_frame,
            contact_tol=0.001,
            elem_a=front_front_wheel,
            elem_b=track_front_flange,
            name="front trolley front wheel bears on front flange when open",
        )
        ctx.expect_contact(
            gate_leaf,
            fixed_frame,
            contact_tol=0.001,
            elem_a=front_rear_wheel,
            elem_b=track_rear_flange,
            name="front trolley rear wheel bears on rear flange when open",
        )
        ctx.expect_gap(
            fixed_frame,
            gate_leaf,
            axis="y",
            positive_elem=guide_front_rail,
            negative_elem=guide_shoe_front,
            min_gap=0.010,
            max_gap=0.020,
            name="front guide shoe clears front rail with controlled side gap when open",
        )
        ctx.expect_gap(
            gate_leaf,
            fixed_frame,
            axis="y",
            positive_elem=guide_shoe_front,
            negative_elem=guide_rear_rail,
            min_gap=0.010,
            max_gap=0.020,
            name="front guide shoe clears rear rail with controlled side gap when open",
        )
        ctx.expect_gap(
            fixed_frame,
            gate_leaf,
            axis="x",
            positive_elem=open_top_stop,
            negative_elem=open_bumper,
            max_gap=0.001,
            max_penetration=1e-6,
            name="open over-travel stop arrests the front bumper",
        )
        ctx.expect_gap(
            gate_leaf,
            fixed_frame,
            axis="x",
            positive_elem=latch_block,
            negative_elem=keeper_block,
            min_gap=1.30,
            name="open gate pulls the latch well clear of the keeper",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
