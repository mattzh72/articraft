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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer")

    body_enamel = model.material("body_enamel", rgba=(0.94, 0.20, 0.18, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.87, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")

    base_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.34, 0.22, 0.065, corner_segments=5),
            0.048,
        ),
        "base_shell",
    )
    base.visual(base_shell, material=body_enamel, name="base_shell")

    chrome_skirt = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.354, 0.234, 0.069, corner_segments=5),
            0.006,
        ),
        "base_chrome_skirt",
    )
    base.visual(chrome_skirt, material=chrome, name="base_chrome_skirt")

    def xy_section(width: float, depth: float, radius: float, z: float, x_shift: float) -> list[tuple[float, float, float]]:
        return [
            (x + x_shift, y, z)
            for x, y in rounded_rect_profile(width, depth, radius, corner_segments=5)
        ]

    column_geom = section_loft(
        [
            xy_section(0.12, 0.10, 0.034, 0.048, -0.088),
            xy_section(0.11, 0.11, 0.038, 0.145, -0.090),
            xy_section(0.10, 0.12, 0.040, 0.238, -0.086),
            xy_section(0.092, 0.105, 0.034, 0.305, -0.078),
        ]
    )
    base.visual(
        mesh_from_geometry(column_geom, "base_column"),
        material=body_enamel,
        name="base_column",
    )

    base.visual(
        Box((0.19, 0.145, 0.012)),
        origin=Origin(xyz=(0.102, 0.0, 0.054)),
        material=chrome,
        name="slide_deck",
    )
    base.visual(
        Box((0.192, 0.149, 0.004)),
        origin=Origin(xyz=(0.154, 0.0, 0.051)),
        material=chrome,
        name="slide_deck_cap",
    )
    base.visual(
        Box((0.038, 0.086, 0.006)),
        origin=Origin(xyz=(-0.078, 0.0, 0.305)),
        material=chrome,
        name="rear_hinge_cap",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.086, 0.064, 0.247), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="speed_lever_bezel",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(-0.090, -0.064, 0.246), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="head_lock_bezel",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.35, 0.24, 0.31)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        Box((0.126, 0.126, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=chrome,
        name="carriage_plate",
    )
    bowl.visual(
        Box((0.126, 0.126, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_trim,
        name="carriage_trim",
    )
    bowl.visual(
        Cylinder(radius=0.043, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=chrome,
        name="bowl_pedestal",
    )

    bowl_outer = [
        (0.026, 0.000),
        (0.058, 0.010),
        (0.094, 0.048),
        (0.108, 0.098),
        (0.114, 0.142),
        (0.118, 0.156),
    ]
    bowl_inner = [
        (0.000, 0.006),
        (0.052, 0.014),
        (0.087, 0.050),
        (0.100, 0.098),
        (0.106, 0.146),
    ]
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                bowl_outer,
                bowl_inner,
                segments=36,
                end_cap="round",
                lip_samples=8,
            ),
            "mixing_bowl_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=chrome,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.0, 0.122, 0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_upper_strut",
    )
    bowl.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.0, 0.122, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_lower_strut",
    )
    bowl.visual(
        Cylinder(radius=0.0065, length=0.054),
        origin=Origin(xyz=(0.0, 0.138, 0.101)),
        material=chrome,
        name="handle_grip",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=0.19),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    head = model.part("head")

    def yz_section(
        x_pos: float,
        width: float,
        height: float,
        radius: float,
        z_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y, z + z_shift)
            for z, y in rounded_rect_profile(height, width, radius, corner_segments=5)
        ]

    head_geom = section_loft(
        [
            yz_section(0.060, 0.072, 0.066, 0.020, 0.034),
            yz_section(0.125, 0.124, 0.122, 0.040, 0.044),
            yz_section(0.196, 0.116, 0.112, 0.038, 0.043),
            yz_section(0.252, 0.086, 0.080, 0.028, 0.030),
        ]
    )
    head.visual(
        Box((0.072, 0.072, 0.030)),
        origin=Origin(xyz=(0.036, 0.0, 0.015)),
        material=body_enamel,
        name="rear_hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="rear_hinge_barrel",
    )
    head.visual(
        mesh_from_geometry(head_geom, "head_shell"),
        material=body_enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.031, length=0.032),
        origin=Origin(xyz=(0.236, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="front_chrome_band",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.046),
        origin=Origin(xyz=(0.192, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="drive_housing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.26, 0.14, 0.15)),
        mass=4.2,
        origin=Origin(xyz=(0.135, 0.0, 0.015)),
    )

    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=chrome,
        name="whisk_shaft",
    )
    whisk.visual(
        Cylinder(radius=0.0115, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.067)),
        material=chrome,
        name="whisk_ring",
    )

    whisk_wire_mesh = None
    loop_count = 6
    for index in range(loop_count):
        angle = (2.0 * math.pi * index) / loop_count
        c = math.cos(angle)
        s = math.sin(angle)
        path = [
            (0.011 * c, 0.011 * s, -0.061),
            (0.024 * c, 0.024 * s, -0.085),
            (0.046 * c, 0.046 * s, -0.127),
            (0.062 * c, 0.062 * s, -0.163),
            (0.0, 0.0, -0.195),
            (-0.062 * c, -0.062 * s, -0.163),
            (-0.046 * c, -0.046 * s, -0.127),
            (-0.024 * c, -0.024 * s, -0.085),
            (-0.011 * c, -0.011 * s, -0.061),
        ]
        wire = tube_from_spline_points(
            path,
            radius=0.0017,
            samples_per_segment=10,
            radial_segments=8,
            cap_ends=True,
        )
        whisk_wire_mesh = wire if whisk_wire_mesh is None else whisk_wire_mesh.merge(wire)

    whisk.visual(
        mesh_from_geometry(whisk_wire_mesh, "whisk_wires"),
        material=chrome,
        name="whisk_wires",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.07, length=0.18),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="speed_pivot",
    )
    speed_control.visual(
        Box((0.040, 0.008, 0.012)),
        origin=Origin(xyz=(0.022, 0.013, 0.0)),
        material=dark_trim,
        name="speed_arm",
    )
    speed_control.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(xyz=(0.040, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="speed_grip",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.050, 0.020, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(0.022, 0.010, 0.0)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Cylinder(radius=0.0085, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lock_cap",
    )
    head_lock.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lock_stem",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.020)),
        mass=0.03,
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.102, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.18, lower=0.0, upper=0.060),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.078, 0.0, 0.324)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.184, 0.0, -0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=24.0),
    )
    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.086, 0.069, 0.247)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=math.radians(-24.0),
            upper=math.radians(24.0),
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.090, -0.070, 0.246)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_tilt = object_model.get_articulation("base_to_head")
    whisk_drive = object_model.get_articulation("head_to_whisk")
    speed_lever = object_model.get_articulation("base_to_speed_control")
    lock_button = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "canonical articulation tree",
        bowl_slide.parent == "base"
        and bowl_slide.child == "bowl"
        and bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.parent == "base"
        and head_tilt.child == "head"
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and whisk_drive.parent == "head"
        and whisk_drive.child == "whisk"
        and whisk_drive.articulation_type == ArticulationType.CONTINUOUS
        and speed_lever.parent == "base"
        and speed_lever.child == "speed_control"
        and speed_lever.articulation_type == ArticulationType.REVOLUTE
        and lock_button.parent == "base"
        and lock_button.child == "head_lock"
        and lock_button.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"bowl={bowl_slide.parent}->{bowl_slide.child}/{bowl_slide.articulation_type}, "
            f"head={head_tilt.parent}->{head_tilt.child}/{head_tilt.articulation_type}, "
            f"whisk={whisk_drive.parent}->{whisk_drive.child}/{whisk_drive.articulation_type}, "
            f"speed={speed_lever.parent}->{speed_lever.child}/{speed_lever.articulation_type}, "
            f"lock={lock_button.parent}->{lock_button.child}/{lock_button.articulation_type}"
        ),
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="slide_deck",
        min_gap=0.0,
        max_gap=0.002,
        name="bowl carriage sits on slide deck",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        elem_a="carriage_plate",
        elem_b="slide_deck",
        min_overlap=0.10,
        name="bowl carriage remains captured over deck footprint",
    )
    ctx.expect_overlap(
        whisk,
        bowl,
        axes="xy",
        elem_a="whisk_wires",
        elem_b="bowl_shell",
        min_overlap=0.08,
        name="whisk stays centered over bowl",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="head_shell",
        negative_elem="bowl_shell",
        min_gap=0.010,
        name="head clears bowl rim",
    )

    bowl_rest = ctx.part_world_position(bowl)
    whisk_rest = ctx.part_world_position(whisk)
    speed_grip_rest = ctx.part_element_world_aabb(speed_control, elem="speed_grip")
    lock_cap_rest = ctx.part_element_world_aabb(head_lock, elem="lock_cap")

    slide_upper = bowl_slide.motion_limits.upper
    tilt_upper = head_tilt.motion_limits.upper
    lever_upper = speed_lever.motion_limits.upper
    lock_upper = lock_button.motion_limits.upper

    with ctx.pose({bowl_slide: slide_upper}):
        bowl_extended = ctx.part_world_position(bowl)
        ctx.expect_overlap(
            bowl,
            base,
            axes="x",
            elem_a="carriage_plate",
            elem_b="slide_deck",
            min_overlap=0.060,
            name="extended bowl carriage keeps insertion on deck",
        )
    ctx.check(
        "bowl slides forward",
        bowl_rest is not None
        and bowl_extended is not None
        and bowl_extended[0] > bowl_rest[0] + 0.045,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    with ctx.pose({head_tilt: tilt_upper}):
        whisk_raised = ctx.part_world_position(whisk)
    ctx.check(
        "tilt head raises whisk",
        whisk_rest is not None
        and whisk_raised is not None
        and whisk_raised[2] > whisk_rest[2] + 0.05,
        details=f"rest={whisk_rest}, raised={whisk_raised}",
    )

    with ctx.pose({speed_lever: lever_upper}):
        speed_grip_raised = ctx.part_element_world_aabb(speed_control, elem="speed_grip")
    ctx.check(
        "speed lever swings through an arc",
        speed_grip_rest is not None
        and speed_grip_raised is not None
        and speed_grip_raised[0][2] > speed_grip_rest[0][2] + 0.006,
        details=f"rest={speed_grip_rest}, raised={speed_grip_raised}",
    )

    with ctx.pose({lock_button: lock_upper}):
        lock_cap_pressed = ctx.part_element_world_aabb(head_lock, elem="lock_cap")
    ctx.check(
        "head lock button presses inward",
        lock_cap_rest is not None
        and lock_cap_pressed is not None
        and lock_cap_pressed[1][1] > lock_cap_rest[1][1] + 0.0025,
        details=f"rest={lock_cap_rest}, pressed={lock_cap_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
