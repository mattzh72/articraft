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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer")

    satin_white = model.material("satin_white", rgba=(0.95, 0.95, 0.93, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.87, 0.89, 0.91, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.17, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.72, 0.73, 0.75, 1.0))

    def xy_section(
        center_x: float,
        center_y: float,
        z: float,
        width: float,
        depth: float,
        radius: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (center_x + px, center_y + py, z)
            for px, py in rounded_rect_profile(width, depth, radius)
        ]

    def yz_section(
        x: float,
        width_y: float,
        height_z: float,
        radius: float,
        center_z: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + center_z)
            for z, y in rounded_rect_profile(height_z, width_y, radius)
        ]

    base = model.part("base")

    base_plinth = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.34, 0.22, 0.05), 0.044),
        "mixer_base_plinth",
    )
    base.visual(
        base_plinth,
        origin=Origin(xyz=(0.02, 0.0, 0.022)),
        material=satin_white,
        name="base_plinth",
    )

    pedestal_shell = mesh_from_geometry(
        section_loft(
            [
                xy_section(-0.085, 0.0, 0.044, 0.130, 0.122, 0.034),
                xy_section(-0.080, 0.0, 0.115, 0.104, 0.110, 0.030),
                xy_section(-0.078, 0.0, 0.185, 0.080, 0.092, 0.026),
                xy_section(-0.082, 0.0, 0.247, 0.060, 0.082, 0.022),
            ]
        ),
        "mixer_pedestal_shell",
    )
    base.visual(
        pedestal_shell,
        material=satin_white,
        name="pedestal_shell",
    )
    base.visual(
        Box((0.028, 0.028, 0.040)),
        origin=Origin(xyz=(-0.106, 0.055, 0.224)),
        material=soft_gray,
        name="left_hinge_support",
    )
    base.visual(
        Box((0.028, 0.028, 0.040)),
        origin=Origin(xyz=(-0.106, -0.055, 0.224)),
        material=soft_gray,
        name="right_hinge_support",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(-0.094, 0.055, 0.247), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_gray,
        name="left_hinge_pin_boss",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(-0.094, -0.055, 0.247), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_gray,
        name="right_hinge_pin_boss",
    )
    base.visual(
        Box((0.210, 0.026, 0.016)),
        origin=Origin(xyz=(0.095, 0.049, 0.052)),
        material=soft_gray,
        name="left_guide_rail",
    )
    base.visual(
        Box((0.210, 0.026, 0.016)),
        origin=Origin(xyz=(0.095, -0.049, 0.052)),
        material=soft_gray,
        name="right_guide_rail",
    )
    base.visual(
        Box((0.054, 0.022, 0.006)),
        origin=Origin(xyz=(0.092, -0.074, 0.047)),
        material=soft_gray,
        name="lock_track",
    )
    base.visual(
        Box((0.060, 0.010, 0.008)),
        origin=Origin(xyz=(0.092, -0.087, 0.048)),
        material=dark_trim,
        name="lock_track_side",
    )
    base.visual(
        Box((0.090, 0.098, 0.010)),
        origin=Origin(xyz=(-0.050, 0.0, 0.230)),
        material=soft_gray,
        name="head_rest_pad",
    )
    base.visual(
        Box((0.050, 0.025, 0.050)),
        origin=Origin(xyz=(0.055, 0.1025, 0.030)),
        material=satin_white,
        name="selector_pod",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.27)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.140, 0.036, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, 0.007)),
        material=soft_gray,
        name="slide_block",
    )
    bowl_carriage.visual(
        Box((0.160, 0.106, 0.010)),
        origin=Origin(xyz=(0.082, 0.0, 0.030)),
        material=satin_white,
        name="saddle_plate",
    )
    bowl_carriage.visual(
        Box((0.018, 0.026, 0.032)),
        origin=Origin(xyz=(0.018, 0.028, 0.016)),
        material=satin_white,
        name="left_saddle_riser",
    )
    bowl_carriage.visual(
        Box((0.018, 0.026, 0.032)),
        origin=Origin(xyz=(0.018, -0.028, 0.016)),
        material=satin_white,
        name="right_saddle_riser",
    )
    bowl_carriage.visual(
        Box((0.024, 0.082, 0.024)),
        origin=Origin(xyz=(0.146, 0.0, 0.018)),
        material=satin_white,
        name="front_yoke",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.17, 0.11, 0.04)),
        mass=1.8,
        origin=Origin(xyz=(0.080, 0.0, 0.020)),
    )

    bowl = model.part("bowl")
    bowl_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.026, 0.000),
                (0.048, 0.006),
                (0.086, 0.026),
                (0.103, 0.076),
                (0.108, 0.118),
                (0.110, 0.132),
            ],
            [
                (0.012, 0.006),
                (0.041, 0.012),
                (0.080, 0.030),
                (0.096, 0.078),
                (0.100, 0.118),
                (0.102, 0.128),
            ],
            segments=64,
        ),
        "mixer_bowl_shell",
    )
    bowl.visual(
        bowl_shell,
        material=polished_steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.132),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
    )

    head = model.part("head")
    head_shell_geom = section_loft(
        [
            yz_section(0.010, 0.084, 0.050, 0.020, center_z=0.038),
            yz_section(0.088, 0.148, 0.100, 0.038, center_z=0.066),
            yz_section(0.186, 0.134, 0.092, 0.034, center_z=0.060),
            yz_section(0.260, 0.094, 0.052, 0.020, center_z=0.038),
        ]
    )
    head_shell_geom.translate(0.000, 0.0, 0.000)
    head.visual(
        mesh_from_geometry(head_shell_geom, "mixer_head_shell"),
        material=satin_white,
        name="head_shell",
    )
    head.visual(
        Box((0.060, 0.064, 0.030)),
        origin=Origin(xyz=(0.200, 0.0, 0.021)),
        material=satin_white,
        name="nose_mount",
    )
    head.visual(
        Box((0.034, 0.022, 0.018)),
        origin=Origin(xyz=(-0.002, 0.055, 0.017)),
        material=soft_gray,
        name="left_hinge_leaf",
    )
    head.visual(
        Box((0.034, 0.022, 0.018)),
        origin=Origin(xyz=(-0.002, -0.055, 0.017)),
        material=soft_gray,
        name="right_hinge_leaf",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.205, 0.0, 0.006)),
        material=soft_gray,
        name="drive_collar",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.29, 0.16, 0.17)),
        mass=4.6,
        origin=Origin(xyz=(0.145, 0.0, 0.020)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="selector_dial",
    )
    speed_selector.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_gray,
        name="selector_hub",
    )
    speed_selector.visual(
        Box((0.024, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.019, 0.013)),
        material=soft_gray,
        name="selector_pointer",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.030, 0.022, 0.040)),
        mass=0.12,
    )

    lock_control = model.part("lock_control")
    lock_control.visual(
        Box((0.020, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_trim,
        name="lock_slider",
    )
    lock_control.visual(
        Box((0.010, 0.010, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.014)),
        material=soft_gray,
        name="lock_thumb_ridge",
    )
    lock_control.inertial = Inertial.from_geometry(
        Box((0.022, 0.016, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.001, 0.0, 0.009)),
    )

    whisk = model.part("whisk")
    whisk_geom = CylinderGeometry(radius=0.0046, height=0.038).translate(0.0, 0.0, -0.019)
    whisk_geom.merge(CylinderGeometry(radius=0.0085, height=0.020).translate(0.0, 0.0, -0.045))
    whisk_geom.merge(CylinderGeometry(radius=0.0115, height=0.020).translate(0.0, 0.0, -0.066))
    for loop_index in range(8):
        angle = loop_index * math.pi / 8.0
        c = math.cos(angle)
        s = math.sin(angle)
        whisk_geom.merge(
            tube_from_spline_points(
                [
                    (0.009 * c, 0.009 * s, -0.050),
                    (0.019 * c, 0.019 * s, -0.066),
                    (0.034 * c, 0.034 * s, -0.096),
                    (0.048 * c, 0.048 * s, -0.125),
                    (0.0, 0.0, -0.144),
                    (-0.048 * c, -0.048 * s, -0.125),
                    (-0.034 * c, -0.034 * s, -0.096),
                    (-0.019 * c, -0.019 * s, -0.066),
                    (-0.009 * c, -0.009 * s, -0.050),
                ],
                radius=0.0014,
                samples_per_segment=18,
                radial_segments=14,
            )
        )
    whisk.visual(
        mesh_from_geometry(whisk_geom, "mixer_wire_whisk"),
        material=polished_steel,
        name="whisk_shell",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.150),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
    )

    bowl_slide = model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.035, 0.0, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.10,
            lower=0.0,
            upper=0.045,
        ),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_carriage,
        child=bowl,
        origin=Origin(xyz=(0.080, 0.0, 0.035)),
    )
    head_tilt = model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.090, 0.0, 0.247)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    whisk_spin = model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    selector_rotate = model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.055, 0.115, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=1.6,
            lower=-0.75,
            upper=0.75,
        ),
    )
    lock_slide = model.articulation(
        "base_to_lock_control",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_control,
        origin=Origin(xyz=(0.078, -0.074, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.06,
            lower=0.0,
            upper=0.018,
        ),
    )

    model.meta["primary_articulations"] = {
        "bowl_slide": bowl_slide.name,
        "head_tilt": head_tilt.name,
        "whisk_spin": whisk_spin.name,
        "selector_rotate": selector_rotate.name,
        "lock_slide": lock_slide.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    bowl_carriage = object_model.get_part("bowl_carriage")
    whisk = object_model.get_part("whisk")
    speed_selector = object_model.get_part("speed_selector")
    lock_control = object_model.get_part("lock_control")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    whisk_spin = object_model.get_articulation("head_to_whisk")
    selector_rotate = object_model.get_articulation("base_to_speed_selector")
    lock_slide = object_model.get_articulation("base_to_lock_control")

    ctx.check(
        "stand mixer articulation set is present",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and whisk_spin.articulation_type == ArticulationType.CONTINUOUS
        and selector_rotate.articulation_type == ArticulationType.REVOLUTE
        and lock_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"bowl={bowl_slide.articulation_type}, head={head_tilt.articulation_type}, "
            f"whisk={whisk_spin.articulation_type}, selector={selector_rotate.articulation_type}, "
            f"lock={lock_slide.articulation_type}"
        ),
    )

    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        margin=0.004,
        name="whisk stays centered inside bowl footprint at rest",
    )
    ctx.expect_gap(
        lock_control,
        base,
        axis="z",
        positive_elem="lock_slider",
        negative_elem="lock_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="lock slider sits on the base track",
    )
    ctx.expect_gap(
        "head",
        base,
        axis="z",
        positive_elem="left_hinge_leaf",
        negative_elem="left_hinge_pin_boss",
        max_gap=0.0005,
        max_penetration=0.0,
        name="closed head hinge leaf bears on the hinge pin boss",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        bowl_extended = ctx.part_world_position(bowl)
        ctx.check(
            "bowl carriage extends forward",
            bowl_rest is not None
            and bowl_extended is not None
            and bowl_extended[0] > bowl_rest[0] + 0.03,
            details=f"rest={bowl_rest}, extended={bowl_extended}",
        )
        ctx.expect_overlap(
            bowl_carriage,
            base,
            axes="x",
            elem_a="slide_block",
            elem_b="left_guide_rail",
            min_overlap=0.12,
            name="carriage slide block remains retained in the guide rail span",
        )

    whisk_rest = ctx.part_world_position(whisk)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        whisk_open = ctx.part_world_position(whisk)
        ctx.check(
            "tilting head lifts whisk upward",
            whisk_rest is not None
            and whisk_open is not None
            and whisk_open[2] > whisk_rest[2] + 0.10,
            details=f"rest={whisk_rest}, open={whisk_open}",
        )
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            min_gap=0.020,
            name="opened head lifts whisk clear of bowl rim",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
