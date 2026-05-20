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
    model = ArticulatedObject(name="rugged_utility_turnstile", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.27, 0.30, 0.32, 1.0))
    rotor_paint = model.material("rotor_paint", rgba=(0.78, 0.67, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.18, 0.20, 1.0))
    zinc_hardware = model.material("zinc_hardware", rgba=(0.63, 0.65, 0.68, 1.0))
    molded_black = model.material("molded_black", rgba=(0.10, 0.11, 0.12, 1.0))

    def _midpoint(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    def _distance(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def _rpy_for_cylinder(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        length_xy = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(length_xy, dz)
        return (0.0, pitch, yaw)

    def _add_member(
        part_obj,
        a: tuple[float, float, float],
        b: tuple[float, float, float],
        radius: float,
        material,
        *,
        name: str | None = None,
    ) -> None:
        part_obj.visual(
            Cylinder(radius=radius, length=_distance(a, b)),
            origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    def _add_bolt_circle(
        part_obj,
        *,
        radius: float,
        z: float,
        count: int,
        bolt_radius: float,
        bolt_length: float,
        material,
        prefix: str,
    ) -> None:
        for index in range(count):
            angle = (2.0 * math.pi * index) / count
            part_obj.visual(
                Cylinder(radius=bolt_radius, length=bolt_length),
                origin=Origin(
                    xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
                ),
                material=material,
                name=f"{prefix}_{index:02d}",
            )

    base_frame = model.part("base_frame")
    base_frame.inertial = Inertial.from_geometry(
        Box((2.20, 2.20, 2.30)),
        mass=340.0,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
    )

    for y_pos, visual_name in ((0.72, "base_front_rail"), (-0.72, "base_rear_rail")):
        base_frame.visual(
            Box((1.56, 0.12, 0.10)),
            origin=Origin(xyz=(0.0, y_pos, 0.05)),
            material=frame_paint,
            name=visual_name,
        )
    for x_pos, visual_name in ((0.72, "base_right_rail"), (-0.72, "base_left_rail")):
        base_frame.visual(
            Box((0.12, 1.44, 0.10)),
            origin=Origin(xyz=(x_pos, 0.0, 0.05)),
            material=frame_paint,
            name=visual_name,
        )

    for x_pos, visual_name in ((0.42, "pedestal_cross_x_pos"), (-0.42, "pedestal_cross_x_neg")):
        base_frame.visual(
            Box((0.48, 0.16, 0.14)),
            origin=Origin(xyz=(x_pos, 0.0, 0.15)),
            material=frame_paint,
            name=visual_name,
        )
    for y_pos, visual_name in ((0.42, "pedestal_cross_y_pos"), (-0.42, "pedestal_cross_y_neg")):
        base_frame.visual(
            Box((0.16, 0.48, 0.14)),
            origin=Origin(xyz=(0.0, y_pos, 0.15)),
            material=frame_paint,
            name=visual_name,
        )

    for x_pos in (-0.72, 0.72):
        for y_pos in (-0.72, 0.72):
            base_frame.visual(
                Box((0.12, 0.12, 2.06)),
                origin=Origin(xyz=(x_pos, y_pos, 1.13)),
                material=frame_paint,
                name=f"corner_post_{'p' if x_pos > 0 else 'n'}x_{'p' if y_pos > 0 else 'n'}y",
            )

    for y_pos, visual_name in ((0.72, "top_front_rail"), (-0.72, "top_rear_rail")):
        base_frame.visual(
            Box((1.56, 0.12, 0.10)),
            origin=Origin(xyz=(0.0, y_pos, 2.21)),
            material=frame_paint,
            name=visual_name,
        )
    for x_pos, visual_name in ((0.72, "top_right_rail"), (-0.72, "top_left_rail")):
        base_frame.visual(
            Box((0.12, 1.56, 0.10)),
            origin=Origin(xyz=(x_pos, 0.0, 2.21)),
            material=frame_paint,
            name=visual_name,
        )

    for y_pos, visual_name in ((0.72, "mid_front_guard"), (-0.72, "mid_rear_guard")):
        base_frame.visual(
            Box((1.56, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, y_pos, 1.10)),
            material=dark_steel,
            name=visual_name,
        )

    for x_pos, visual_name in ((0.435, "top_cross_x_pos"), (-0.435, "top_cross_x_neg")):
        base_frame.visual(
            Box((0.46, 0.12, 0.08)),
            origin=Origin(xyz=(x_pos, 0.0, 2.13)),
            material=dark_steel,
            name=visual_name,
        )
    for y_pos, visual_name in ((0.435, "top_cross_y_pos"), (-0.435, "top_cross_y_neg")):
        base_frame.visual(
            Box((0.12, 0.46, 0.08)),
            origin=Origin(xyz=(0.0, y_pos, 2.13)),
            material=dark_steel,
            name=visual_name,
        )

    base_frame.visual(
        Cylinder(radius=0.18, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=dark_steel,
        name="pedestal_body",
    )
    base_frame.visual(
        Cylinder(radius=0.14, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        material=zinc_hardware,
        name="pedestal_cap",
    )
    base_frame.visual(
        Cylinder(radius=0.17, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 2.065)),
        material=zinc_hardware,
        name="top_bearing_plate",
    )
    base_frame.visual(
        Cylinder(radius=0.21, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.145)),
        material=dark_steel,
        name="top_bearing_housing",
    )
    base_frame.visual(
        Cylinder(radius=0.23, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 2.075)),
        material=dark_steel,
        name="top_flange_ring",
    )

    base_frame.visual(
        Box((0.12, 0.56, 2.06)),
        origin=Origin(xyz=(0.825, 0.0, 1.13)),
        material=dark_steel,
        name="service_mast",
    )
    base_frame.visual(
        Box((0.22, 0.46, 0.44)),
        origin=Origin(xyz=(0.94, 0.0, 0.32)),
        material=frame_paint,
        name="lower_cabinet",
    )
    base_frame.visual(
        Box((0.26, 0.46, 0.24)),
        origin=Origin(xyz=(0.96, 0.0, 1.95)),
        material=molded_black,
        name="upper_housing",
    )
    base_frame.visual(
        Box((0.10, 0.38, 1.38)),
        origin=Origin(xyz=(0.89, 0.0, 1.20)),
        material=dark_steel,
        name="housing_neck",
    )

    for x_pos in (-0.66, 0.66):
        for y_pos in (-0.66, 0.66):
            base_frame.visual(
                Box((0.16, 0.16, 0.14)),
                origin=Origin(xyz=(x_pos, y_pos, 0.17)),
                material=dark_steel,
                name=f"base_gusset_{'p' if x_pos > 0 else 'n'}x_{'p' if y_pos > 0 else 'n'}y",
            )
            base_frame.visual(
                Box((0.16, 0.16, 0.14)),
                origin=Origin(xyz=(x_pos, y_pos, 2.14)),
                material=dark_steel,
                name=f"top_gusset_{'p' if x_pos > 0 else 'n'}x_{'p' if y_pos > 0 else 'n'}y",
            )

    for x_pos in (-0.66, 0.66):
        for y_pos in (-0.66, 0.66):
            base_frame.visual(
                Cylinder(radius=0.012, length=0.03),
                origin=Origin(xyz=(x_pos, y_pos, 0.085)),
                material=zinc_hardware,
                name=f"anchor_bolt_{'p' if x_pos > 0 else 'n'}x_{'p' if y_pos > 0 else 'n'}y",
            )

    _add_bolt_circle(
        base_frame,
        radius=0.10,
        z=0.300,
        count=6,
        bolt_radius=0.006,
        bolt_length=0.020,
        material=zinc_hardware,
        prefix="pedestal_flange_bolt",
    )
    _add_bolt_circle(
        base_frame,
        radius=0.14,
        z=2.065,
        count=8,
        bolt_radius=0.006,
        bolt_length=0.020,
        material=zinc_hardware,
        prefix="upper_flange_bolt",
    )

    for z_pos, name_prefix in ((0.13, "service_hinge_lower"), (0.35, "service_hinge_upper")):
        base_frame.visual(
            Cylinder(radius=0.016, length=0.02),
            origin=Origin(xyz=(1.05, 0.0, z_pos)),
            material=dark_steel,
            name=name_prefix,
        )

    for y_pos, name_prefix in ((-0.09, "hatch_hinge_left"), (0.09, "hatch_hinge_right")):
        base_frame.visual(
            Cylinder(radius=0.015, length=0.06),
            origin=Origin(xyz=(1.09, y_pos, 2.07), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=name_prefix,
        )

    rotor_assembly = model.part("rotor_assembly")
    rotor_assembly.inertial = Inertial.from_geometry(
        Box((1.10, 1.10, 1.76)),
        mass=115.0,
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
    )

    rotor_assembly.visual(
        Cylinder(radius=0.14, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=zinc_hardware,
        name="lower_thrust_collar",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.11, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="lower_bearing_drum",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.08, length=1.70),
        origin=Origin(xyz=(0.0, 0.0, 0.87)),
        material=dark_steel,
        name="central_spindle",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.11, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.68)),
        material=dark_steel,
        name="upper_guide_collar",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.14, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 1.73)),
        material=zinc_hardware,
        name="upper_thrust_disk",
    )

    for z_pos, name_prefix in ((0.20, "lower_hub"), (0.88, "mid_hub"), (1.56, "upper_hub")):
        rotor_assembly.visual(
            Cylinder(radius=0.12, length=0.06),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=rotor_paint,
            name=name_prefix,
        )

    arm_inner_radius = 0.12
    arm_outer_radius = 0.56
    outer_post_radius = 0.038
    arm_radius = 0.030

    for wing_index in range(3):
        angle = (math.pi / 2.0) + (2.0 * math.pi * wing_index) / 3.0
        c = math.cos(angle)
        s = math.sin(angle)

        def radial_point(radius: float, z: float) -> tuple[float, float, float]:
            return (radius * c, radius * s, z)

        _add_member(
            rotor_assembly,
            radial_point(arm_outer_radius, 0.12),
            radial_point(arm_outer_radius, 1.68),
            outer_post_radius,
            rotor_paint,
            name=f"wing_{wing_index}_outer_post",
        )
        _add_member(
            rotor_assembly,
            radial_point(arm_inner_radius, 0.20),
            radial_point(arm_outer_radius, 0.20),
            arm_radius,
            rotor_paint,
            name=f"wing_{wing_index}_lower_arm",
        )
        _add_member(
            rotor_assembly,
            radial_point(arm_inner_radius, 0.88),
            radial_point(arm_outer_radius, 0.88),
            arm_radius,
            rotor_paint,
            name=f"wing_{wing_index}_mid_arm",
        )
        _add_member(
            rotor_assembly,
            radial_point(arm_inner_radius, 1.56),
            radial_point(arm_outer_radius, 1.56),
            arm_radius,
            rotor_paint,
            name=f"wing_{wing_index}_upper_arm",
        )
        _add_member(
            rotor_assembly,
            radial_point(0.18, 0.34),
            radial_point(arm_outer_radius - 0.01, 0.72),
            0.022,
            dark_steel,
        )
        _add_member(
            rotor_assembly,
            radial_point(0.18, 1.42),
            radial_point(arm_outer_radius - 0.01, 1.04),
            0.022,
            dark_steel,
        )
        for z_pos, collar_name in ((0.20, "lower_joint"), (0.88, "mid_joint"), (1.56, "upper_joint")):
            rotor_assembly.visual(
                Cylinder(radius=0.052, length=0.07),
                origin=Origin(xyz=(arm_outer_radius * c, arm_outer_radius * s, z_pos)),
                material=dark_steel,
                name=f"wing_{wing_index}_{collar_name}",
            )

    _add_bolt_circle(
        rotor_assembly,
        radius=0.115,
        z=0.23,
        count=6,
        bolt_radius=0.0055,
        bolt_length=0.016,
        material=zinc_hardware,
        prefix="lower_hub_bolt",
    )
    _add_bolt_circle(
        rotor_assembly,
        radius=0.115,
        z=0.91,
        count=6,
        bolt_radius=0.0055,
        bolt_length=0.016,
        material=zinc_hardware,
        prefix="mid_hub_bolt",
    )
    _add_bolt_circle(
        rotor_assembly,
        radius=0.115,
        z=1.59,
        count=6,
        bolt_radius=0.0055,
        bolt_length=0.016,
        material=zinc_hardware,
        prefix="upper_hub_bolt",
    )

    service_panel = model.part("service_panel")
    service_panel.inertial = Inertial.from_geometry(
        Box((0.05, 0.34, 0.30)),
        mass=6.0,
        origin=Origin(xyz=(0.025, -0.17, 0.15)),
    )
    service_panel.visual(
        Cylinder(radius=0.014, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_steel,
        name="panel_barrel",
    )
    service_panel.visual(
        Box((0.016, 0.34, 0.30)),
        origin=Origin(xyz=(0.022, -0.17, 0.15)),
        material=molded_black,
        name="panel_leaf",
    )
    service_panel.visual(
        Box((0.05, 0.09, 0.03)),
        origin=Origin(xyz=(0.034, -0.25, 0.17)),
        material=dark_steel,
        name="panel_handle",
    )
    for y_pos in (-0.28, -0.17, -0.06):
        service_panel.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.028, y_pos, 0.23)),
            material=zinc_hardware,
            name=f"panel_fastener_{abs(int(y_pos * 100)):02d}",
        )

    inspection_hatch = model.part("inspection_hatch")
    inspection_hatch.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 0.03)),
        mass=4.5,
        origin=Origin(xyz=(0.10, 0.0, 0.015)),
    )
    inspection_hatch.visual(
        Cylinder(radius=0.014, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hatch_barrel_support",
    )
    inspection_hatch.visual(
        Box((0.028, 0.10, 0.02)),
        origin=Origin(xyz=(0.018, 0.0, 0.010)),
        material=dark_steel,
        name="hatch_spine",
    )
    inspection_hatch.visual(
        Box((0.20, 0.24, 0.012)),
        origin=Origin(xyz=(0.132, 0.0, 0.006)),
        material=molded_black,
        name="hatch_leaf",
    )
    inspection_hatch.visual(
        Box((0.05, 0.08, 0.03)),
        origin=Origin(xyz=(0.172, 0.0, 0.024)),
        material=dark_steel,
        name="hatch_handle",
    )

    model.articulation(
        "main_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=rotor_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=280.0, velocity=0.85),
    )
    model.articulation(
        "service_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=service_panel,
        origin=Origin(xyz=(1.064, 0.0, 0.09)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "inspection_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=inspection_hatch,
        origin=Origin(xyz=(1.104, 0.0, 2.07)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    rotor_assembly = object_model.get_part("rotor_assembly")
    service_panel = object_model.get_part("service_panel")
    inspection_hatch = object_model.get_part("inspection_hatch")

    main_rotation = object_model.get_articulation("main_rotation")
    service_panel_hinge = object_model.get_articulation("service_panel_hinge")
    inspection_hatch_hinge = object_model.get_articulation("inspection_hatch_hinge")

    pedestal_cap = base_frame.get_visual("pedestal_cap")
    top_bearing_plate = base_frame.get_visual("top_bearing_plate")
    lower_cabinet = base_frame.get_visual("lower_cabinet")
    upper_housing = base_frame.get_visual("upper_housing")
    service_hinge_lower = base_frame.get_visual("service_hinge_lower")
    service_hinge_upper = base_frame.get_visual("service_hinge_upper")
    hatch_hinge_left = base_frame.get_visual("hatch_hinge_left")
    hatch_hinge_right = base_frame.get_visual("hatch_hinge_right")

    lower_thrust_collar = rotor_assembly.get_visual("lower_thrust_collar")
    upper_thrust_disk = rotor_assembly.get_visual("upper_thrust_disk")
    panel_leaf = service_panel.get_visual("panel_leaf")
    panel_barrel = service_panel.get_visual("panel_barrel")
    hatch_leaf = inspection_hatch.get_visual("hatch_leaf")
    hatch_barrel_support = inspection_hatch.get_visual("hatch_barrel_support")

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
        base_frame,
        service_panel,
        elem_a=service_hinge_lower,
        elem_b=panel_barrel,
        reason="The service-panel hinge barrel wraps around the fixed lower hinge pin.",
    )
    ctx.allow_overlap(
        base_frame,
        service_panel,
        elem_a=service_hinge_upper,
        elem_b=panel_barrel,
        reason="The service-panel hinge barrel wraps around the fixed upper hinge pin.",
    )
    ctx.allow_overlap(
        base_frame,
        inspection_hatch,
        elem_a=hatch_hinge_left,
        elem_b=hatch_barrel_support,
        reason="The inspection hatch uses a through-hinge barrel riding on the left hinge pin.",
    )
    ctx.allow_overlap(
        base_frame,
        inspection_hatch,
        elem_a=hatch_hinge_right,
        elem_b=hatch_barrel_support,
        reason="The inspection hatch uses a through-hinge barrel riding on the right hinge pin.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        rotor_assembly,
        base_frame,
        elem_a=lower_thrust_collar,
        elem_b=pedestal_cap,
        name="rotor_lower_bearing_contact",
    )
    ctx.expect_contact(
        rotor_assembly,
        base_frame,
        elem_a=upper_thrust_disk,
        elem_b=top_bearing_plate,
        name="rotor_upper_bearing_contact",
    )
    ctx.expect_contact(
        service_panel,
        base_frame,
        elem_a=panel_barrel,
        elem_b=service_hinge_lower,
        name="service_panel_lower_hinge_contact",
    )
    ctx.expect_contact(
        service_panel,
        base_frame,
        elem_a=panel_barrel,
        elem_b=service_hinge_upper,
        name="service_panel_upper_hinge_contact",
    )
    ctx.expect_contact(
        inspection_hatch,
        base_frame,
        elem_a=hatch_barrel_support,
        elem_b=hatch_hinge_left,
        name="inspection_hatch_left_hinge_contact",
    )
    ctx.expect_contact(
        inspection_hatch,
        base_frame,
        elem_a=hatch_barrel_support,
        elem_b=hatch_hinge_right,
        name="inspection_hatch_right_hinge_contact",
    )

    ctx.expect_gap(
        service_panel,
        base_frame,
        axis="x",
        positive_elem=panel_leaf,
        negative_elem=lower_cabinet,
        min_gap=0.0,
        max_gap=0.040,
        name="service_panel_closed_offset",
    )
    ctx.expect_gap(
        inspection_hatch,
        base_frame,
        axis="z",
        positive_elem=hatch_leaf,
        negative_elem=upper_housing,
        min_gap=0.0,
        max_gap=0.025,
        name="inspection_hatch_closed_offset",
    )
    ctx.expect_within(
        rotor_assembly,
        base_frame,
        axes="xy",
        margin=0.0,
        name="rotor_within_frame_envelope",
    )

    ctx.check(
        "main_rotation_axis_vertical",
        tuple(main_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={main_rotation.axis}",
    )
    ctx.check(
        "service_panel_hinge_axis_vertical",
        tuple(service_panel_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={service_panel_hinge.axis}",
    )
    ctx.check(
        "inspection_hatch_hinge_axis_horizontal",
        tuple(inspection_hatch_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={inspection_hatch_hinge.axis}",
    )

    frame_aabb = ctx.part_world_aabb(base_frame)
    rotor_aabb = ctx.part_world_aabb(rotor_assembly)
    if frame_aabb is not None:
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame_height_realistic",
            2.15 <= frame_height <= 2.35,
            details=f"height={frame_height:.3f}",
        )
    if rotor_aabb is not None:
        rotor_diameter_x = rotor_aabb[1][0] - rotor_aabb[0][0]
        rotor_diameter_y = rotor_aabb[1][1] - rotor_aabb[0][1]
        ctx.check(
            "rotor_diameter_realistic",
            0.95 <= max(rotor_diameter_x, rotor_diameter_y) <= 1.15,
            details=f"dx={rotor_diameter_x:.3f}, dy={rotor_diameter_y:.3f}",
        )

    for pose_index, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi)):
        with ctx.pose({main_rotation: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"rotor_pose_{pose_index}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"rotor_pose_{pose_index}_no_floating")
            ctx.expect_contact(
                rotor_assembly,
                base_frame,
                elem_a=lower_thrust_collar,
                elem_b=pedestal_cap,
                name=f"rotor_pose_{pose_index}_lower_contact",
            )
            ctx.expect_contact(
                rotor_assembly,
                base_frame,
                elem_a=upper_thrust_disk,
                elem_b=top_bearing_plate,
                name=f"rotor_pose_{pose_index}_upper_contact",
            )
            ctx.expect_within(
                rotor_assembly,
                base_frame,
                axes="xy",
                margin=0.0,
                name=f"rotor_pose_{pose_index}_within_frame",
            )

    service_panel_rest = ctx.part_world_aabb(service_panel)
    assert service_panel_rest is not None
    with ctx.pose({service_panel_hinge: 1.20}):
        service_panel_open = ctx.part_world_aabb(service_panel)
        assert service_panel_open is not None
        ctx.check(
            "service_panel_opens_outward",
            service_panel_open[1][0] > service_panel_rest[1][0] + 0.12,
            details=f"rest_max_x={service_panel_rest[1][0]:.3f}, open_max_x={service_panel_open[1][0]:.3f}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="service_panel_open_no_overlap")
        ctx.fail_if_isolated_parts(name="service_panel_open_no_floating")
        ctx.expect_contact(
            service_panel,
            base_frame,
            elem_a=panel_barrel,
            elem_b=service_hinge_lower,
            name="service_panel_open_lower_contact",
        )
        ctx.expect_contact(
            service_panel,
            base_frame,
            elem_a=panel_barrel,
            elem_b=service_hinge_upper,
            name="service_panel_open_upper_contact",
        )

    inspection_hatch_rest = ctx.part_world_aabb(inspection_hatch)
    assert inspection_hatch_rest is not None
    with ctx.pose({inspection_hatch_hinge: 1.00}):
        inspection_hatch_open = ctx.part_world_aabb(inspection_hatch)
        assert inspection_hatch_open is not None
        ctx.check(
            "inspection_hatch_lifts_clear",
            inspection_hatch_open[1][2] > inspection_hatch_rest[1][2] + 0.12,
            details=f"rest_max_z={inspection_hatch_rest[1][2]:.3f}, open_max_z={inspection_hatch_open[1][2]:.3f}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="inspection_hatch_open_no_overlap")
        ctx.fail_if_isolated_parts(name="inspection_hatch_open_no_floating")
        ctx.expect_contact(
            inspection_hatch,
            base_frame,
            elem_a=hatch_barrel_support,
            elem_b=hatch_hinge_left,
            name="inspection_hatch_open_left_contact",
        )
        ctx.expect_contact(
            inspection_hatch,
            base_frame,
            elem_a=hatch_barrel_support,
            elem_b=hatch_hinge_right,
            name="inspection_hatch_open_right_contact",
        )

    with ctx.pose(
        {
            main_rotation: math.pi / 3.0,
            service_panel_hinge: 1.20,
            inspection_hatch_hinge: 1.00,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_service_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_service_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
