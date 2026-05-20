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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satin_stand_mixer")

    satin_body = model.material("satin_body", rgba=(0.80, 0.81, 0.83, 1.0))
    steel = model.material("steel", rgba=(0.87, 0.88, 0.89, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.21, 0.22, 0.24, 1.0))

    def yz_section(
        width_y: float,
        height_z: float,
        radius: float,
        x: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + z_center)
            for z, y in rounded_rect_profile(height_z, width_y, radius)
        ]

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    base = model.part("base")
    heel_mesh = BoxGeometry((0.35, 0.24, 0.03)).translate(0.055, 0.0, 0.015)
    pedestal_mesh = section_loft(
        [
            yz_section(0.07, 0.18, 0.022, -0.095, 0.105),
            yz_section(0.12, 0.22, 0.032, -0.030, 0.150),
            yz_section(0.10, 0.25, 0.030, 0.025, 0.160),
        ]
    )
    base.visual(save_mesh("base_heel", heel_mesh), material=satin_body, name="base_heel")
    base.visual(
        save_mesh("base_pedestal", pedestal_mesh),
        material=satin_body,
        name="base_pedestal",
    )
    base.visual(
        Box((0.09, 0.115, 0.17)),
        origin=Origin(xyz=(0.045, 0.0, 0.115)),
        material=satin_body,
        name="carriage_tower",
    )
    base.visual(
        Box((0.034, 0.024, 0.058)),
        origin=Origin(xyz=(-0.055, 0.067, 0.257)),
        material=satin_body,
        name="rear_hinge_left_cheek",
    )
    base.visual(
        Box((0.034, 0.024, 0.058)),
        origin=Origin(xyz=(-0.055, -0.067, 0.257)),
        material=satin_body,
        name="rear_hinge_right_cheek",
    )
    base.visual(
        Box((0.034, 0.040, 0.040)),
        origin=Origin(xyz=(-0.055, 0.050, 0.240)),
        material=satin_body,
        name="rear_hinge_left_mount",
    )
    base.visual(
        Box((0.034, 0.040, 0.040)),
        origin=Origin(xyz=(-0.055, -0.050, 0.240)),
        material=satin_body,
        name="rear_hinge_right_mount",
    )
    base.visual(
        Box((0.024, 0.004, 0.032)),
        origin=Origin(xyz=(0.045, 0.058, 0.155)),
        material=dark_trim,
        name="selector_bezel",
    )
    base.visual(
        Box((0.036, 0.056, 0.052)),
        origin=Origin(xyz=(-0.056, -0.092, 0.286)),
        material=dark_trim,
        name="lock_guide",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.35, 0.24, 0.33)),
        mass=8.0,
        origin=Origin(xyz=(0.055, 0.0, 0.165)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            yz_section(0.09, 0.045, 0.018, 0.006, 0.055),
            yz_section(0.09, 0.08, 0.022, 0.045, 0.092),
            yz_section(0.15, 0.13, 0.038, 0.200, 0.070),
            yz_section(0.08, 0.09, 0.020, 0.330, 0.040),
        ]
    )
    head.visual(save_mesh("head_shell", head_shell), material=satin_body, name="head_shell")
    head.visual(
        Box((0.040, 0.092, 0.020)),
        origin=Origin(xyz=(0.020, 0.0, 0.054)),
        material=satin_body,
        name="rear_hinge_bridge",
    )
    head.visual(
        Box((0.034, 0.020, 0.072)),
        origin=Origin(xyz=(0.012, 0.045, 0.026)),
        material=satin_body,
        name="rear_hinge_left_lug",
    )
    head.visual(
        Box((0.034, 0.020, 0.072)),
        origin=Origin(xyz=(0.012, -0.045, 0.026)),
        material=satin_body,
        name="rear_hinge_right_lug",
    )
    head.visual(
        Cylinder(radius=0.033, length=0.06),
        origin=Origin(xyz=(0.332, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_body,
        name="head_nose",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.318, 0.0, -0.010)),
        material=dark_trim,
        name="beater_collar",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.16, 0.14)),
        mass=4.0,
        origin=Origin(xyz=(0.170, 0.0, 0.015)),
    )

    carriage = model.part("bowl_carriage")
    carriage.visual(
        Box((0.03, 0.12, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_trim,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.15, 0.13, 0.024)),
        origin=Origin(xyz=(0.075, 0.0, 0.066)),
        material=dark_trim,
        name="carriage_crossarm",
    )
    carriage.visual(
        Cylinder(radius=0.07, length=0.010),
        origin=Origin(xyz=(0.145, 0.0, 0.083)),
        material=steel,
        name="bowl_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.19, 0.13, 0.12)),
        mass=1.2,
        origin=Origin(xyz=(0.080, 0.0, 0.080)),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.050, 0.000),
            (0.067, 0.008),
            (0.095, 0.050),
            (0.108, 0.115),
            (0.113, 0.157),
            (0.116, 0.165),
        ],
        inner_profile=[
            (0.020, 0.007),
            (0.055, 0.017),
            (0.090, 0.055),
            (0.102, 0.120),
            (0.108, 0.158),
        ],
        segments=64,
    )
    bowl.visual(save_mesh("mixing_bowl", bowl_shell), material=steel, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.116, length=0.165),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
    )

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.005, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=steel,
        name="beater_shaft",
    )
    beater.visual(
        Box((0.010, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=steel,
        name="beater_spine",
    )
    beater.visual(
        Box((0.052, 0.007, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.132)),
        material=steel,
        name="beater_blade",
    )
    beater.visual(
        Box((0.007, 0.030, 0.065)),
        origin=Origin(xyz=(0.020, 0.0, -0.103)),
        material=steel,
        name="beater_scraper",
    )
    beater.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, 0.150)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
    )

    selector = model.part("speed_selector")
    selector.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="selector_knob",
    )
    selector.visual(
        Box((0.022, 0.008, 0.046)),
        origin=Origin(xyz=(0.014, 0.0, 0.024)),
        material=dark_trim,
        name="selector_lever",
    )
    selector.inertial = Inertial.from_geometry(
        Box((0.032, 0.016, 0.052)),
        mass=0.08,
        origin=Origin(xyz=(0.008, 0.0, 0.020)),
    )

    lock_control = model.part("lock_control")
    lock_control.visual(
        Box((0.026, 0.010, 0.040)),
        origin=Origin(xyz=(0.013, 0.0, 0.020)),
        material=dark_trim,
        name="lock_slider",
    )
    lock_control.visual(
        Box((0.008, 0.010, 0.020)),
        origin=Origin(xyz=(0.024, -0.005, 0.038)),
        material=dark_trim,
        name="lock_tab",
    )
    lock_control.inertial = Inertial.from_geometry(
        Box((0.030, 0.012, 0.044)),
        mass=0.06,
        origin=Origin(xyz=(0.015, -0.002, 0.022)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.055, 0.0, 0.285)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(63.0),
        ),
    )
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.06,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.145, 0.0, 0.088)),
    )
    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.318, 0.0, -0.019)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=12.0,
        ),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=selector,
        origin=Origin(xyz=(0.045, 0.067, 0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=3.0,
            lower=-0.8,
            upper=0.8,
        ),
    )
    model.articulation(
        "base_to_lock_control",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_control,
        origin=Origin(xyz=(-0.070, -0.125, 0.266)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.04,
            lower=0.0,
            upper=0.008,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    selector = object_model.get_part("speed_selector")
    lock_control = object_model.get_part("lock_control")
    head_tilt = object_model.get_articulation("base_to_head")
    carriage_lift = object_model.get_articulation("base_to_carriage")
    beater_spin = object_model.get_articulation("head_to_beater")
    selector_joint = object_model.get_articulation("base_to_speed_selector")
    lock_joint = object_model.get_articulation("base_to_lock_control")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
        return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5)

    ctx.check(
        "canonical articulation types are preserved",
        head_tilt.articulation_type == ArticulationType.REVOLUTE
        and carriage_lift.articulation_type == ArticulationType.PRISMATIC
        and beater_spin.articulation_type == ArticulationType.CONTINUOUS
        and selector_joint.articulation_type == ArticulationType.REVOLUTE
        and lock_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"tilt={head_tilt.articulation_type}, carriage={carriage_lift.articulation_type}, "
            f"beater={beater_spin.articulation_type}, selector={selector_joint.articulation_type}, "
            f"lock={lock_joint.articulation_type}"
        ),
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({carriage_lift: carriage_lift.motion_limits.upper}):
        bowl_raised = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage raises upward",
        bowl_rest is not None
        and bowl_raised is not None
        and bowl_raised[2] > bowl_rest[2] + 0.012,
        details=f"rest={bowl_rest}, raised={bowl_raised}",
    )

    nose_rest = aabb_center(ctx.part_element_world_aabb(head, elem="head_nose"))
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        nose_open = aabb_center(ctx.part_element_world_aabb(head, elem="head_nose"))
    ctx.check(
        "tilt head lifts the nose upward",
        nose_rest is not None
        and nose_open is not None
        and nose_open[2] > nose_rest[2] + 0.10,
        details=f"rest={nose_rest}, open={nose_open}",
    )

    scraper_rest = aabb_center(ctx.part_element_world_aabb(beater, elem="beater_scraper"))
    with ctx.pose({beater_spin: math.pi / 2.0}):
        scraper_spun = aabb_center(ctx.part_element_world_aabb(beater, elem="beater_scraper"))
    ctx.check(
        "flat beater spins beneath the head",
        scraper_rest is not None
        and scraper_spun is not None
        and abs(scraper_spun[1] - scraper_rest[1]) > 0.012,
        details=f"rest={scraper_rest}, spun={scraper_spun}",
    )

    selector_rest = aabb_center(ctx.part_element_world_aabb(selector, elem="selector_lever"))
    with ctx.pose({selector_joint: selector_joint.motion_limits.upper}):
        selector_turned = aabb_center(ctx.part_element_world_aabb(selector, elem="selector_lever"))
    ctx.check(
        "speed selector rotates through its sweep",
        selector_rest is not None
        and selector_turned is not None
        and abs(selector_turned[0] - selector_rest[0]) > 0.010
        and abs(selector_turned[2] - selector_rest[2]) > 0.010,
        details=f"rest={selector_rest}, turned={selector_turned}",
    )

    lock_rest = ctx.part_world_position(lock_control)
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        lock_open = ctx.part_world_position(lock_control)
    ctx.check(
        "base lock control slides on a short stroke",
        lock_rest is not None
        and lock_open is not None
        and lock_open[0] > lock_rest[0] + 0.006,
        details=f"rest={lock_rest}, open={lock_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
