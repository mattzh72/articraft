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
    model = ArticulatedObject(name="museum_track_spot")

    ceiling_white = model.material("ceiling_white", rgba=(0.93, 0.93, 0.92, 1.0))
    satin_white = model.material("satin_white", rgba=(0.88, 0.88, 0.86, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.72, 0.72, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.80, 0.88, 0.92, 0.40))

    rail = model.part("rail")
    rail.visual(
        Box((1.20, 0.050, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=ceiling_white,
        name="track_backplate",
    )
    rail.visual(
        Box((1.20, 0.036, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=satin_white,
        name="track_top_web",
    )
    rail.visual(
        Box((1.20, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, -0.016, -0.015)),
        material=satin_white,
        name="track_left_wall",
    )
    rail.visual(
        Box((1.20, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.016, -0.015)),
        material=satin_white,
        name="track_right_wall",
    )
    rail.visual(
        Box((1.20, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, -0.028)),
        material=satin_white,
        name="track_left_lip",
    )
    rail.visual(
        Box((1.20, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.010, -0.028)),
        material=satin_white,
        name="track_right_lip",
    )
    rail.visual(
        Box((0.080, 0.048, 0.010)),
        origin=Origin(xyz=(-0.520, 0.0, -0.009)),
        material=warm_grey,
        name="left_end_feed",
    )
    rail.visual(
        Box((0.080, 0.048, 0.010)),
        origin=Origin(xyz=(0.520, 0.0, -0.009)),
        material=warm_grey,
        name="right_end_cap",
    )
    rail.inertial = Inertial.from_geometry(
        Box((1.20, 0.050, 0.035)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.100, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_metal,
        name="runner_block",
    )
    carriage.visual(
        Box((0.050, 0.010, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=dark_metal,
        name="hanger_neck",
    )
    carriage.visual(
        Box((0.040, 0.036, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_metal,
        name="support_crosshead",
    )
    carriage.visual(
        Box((0.060, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, -0.016, 0.005)),
        material=dark_metal,
        name="left_support_strut",
    )
    carriage.visual(
        Box((0.060, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, 0.016, 0.005)),
        material=dark_metal,
        name="right_support_strut",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(-0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="left_bearing_ear",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="right_bearing_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.110, 0.040, 0.070)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    swivel_yoke = model.part("swivel_yoke")
    swivel_yoke.visual(
        Cylinder(radius=0.012, length=0.038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_grey,
        name="spindle",
    )
    swivel_yoke.visual(
        Box((0.020, 0.014, 0.036)),
        origin=Origin(xyz=(-0.010, 0.0, -0.030)),
        material=warm_grey,
        name="drop_stem",
    )
    swivel_yoke.visual(
        Box((0.070, 0.014, 0.012)),
        origin=Origin(xyz=(-0.045, 0.0, -0.047)),
        material=warm_grey,
        name="knuckle_arm",
    )
    swivel_yoke.visual(
        Box((0.028, 0.112, 0.010)),
        origin=Origin(xyz=(-0.064, 0.0, -0.053)),
        material=warm_grey,
        name="yoke_bridge",
    )
    swivel_yoke.visual(
        Box((0.014, 0.008, 0.072)),
        origin=Origin(xyz=(-0.064, -0.050, -0.094)),
        material=warm_grey,
        name="left_cheek",
    )
    swivel_yoke.visual(
        Box((0.014, 0.008, 0.072)),
        origin=Origin(xyz=(-0.064, 0.050, -0.094)),
        material=warm_grey,
        name="right_cheek",
    )
    swivel_yoke.inertial = Inertial.from_geometry(
        Box((0.120, 0.120, 0.130)),
        mass=0.35,
        origin=Origin(xyz=(-0.050, 0.0, -0.060)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.006, length=0.082),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_axle",
    )
    lamp_head.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, -0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.040, length=0.160),
        origin=Origin(xyz=(0.042, 0.0, -0.092)),
        material=satin_white,
        name="main_barrel",
    )
    lamp_head.visual(
        Cylinder(radius=0.043, length=0.005),
        origin=Origin(xyz=(0.040, 0.0, -0.040)),
        material=warm_grey,
        name="cooling_fin_upper",
    )
    lamp_head.visual(
        Cylinder(radius=0.043, length=0.005),
        origin=Origin(xyz=(0.040, 0.0, -0.058)),
        material=warm_grey,
        name="cooling_fin_mid",
    )
    lamp_head.visual(
        Cylinder(radius=0.043, length=0.005),
        origin=Origin(xyz=(0.040, 0.0, -0.076)),
        material=warm_grey,
        name="cooling_fin_lower",
    )
    lamp_head.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.034, 0.0, -0.001)),
        material=warm_grey,
        name="rear_cap",
    )
    lamp_head.visual(
        Cylinder(radius=0.046, length=0.020),
        origin=Origin(xyz=(0.050, 0.0, -0.182)),
        material=dark_metal,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.033, length=0.003),
        origin=Origin(xyz=(0.050, 0.0, -0.1935)),
        material=lens_glass,
        name="front_glass",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.120, 0.100, 0.220)),
        mass=0.85,
        origin=Origin(xyz=(0.040, 0.0, -0.100)),
    )

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.45,
            lower=-0.460,
            upper=0.460,
        ),
    )
    model.articulation(
        "carriage_swivel",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=swivel_yoke,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.4),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_yoke,
        child=lamp_head,
        origin=Origin(xyz=(-0.062, 0.0, -0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    swivel_yoke = object_model.get_part("swivel_yoke")
    lamp_head = object_model.get_part("lamp_head")

    rail_slide = object_model.get_articulation("rail_slide")
    carriage_swivel = object_model.get_articulation("carriage_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    def elem_center(part, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check(
        "all prompt-critical parts exist",
        all(part is not None for part in (rail, carriage, swivel_yoke, lamp_head)),
    )
    ctx.check(
        "all prompt-critical articulations exist",
        all(joint is not None for joint in (rail_slide, carriage_swivel, head_tilt)),
    )

    with ctx.pose({rail_slide: 0.0, carriage_swivel: 0.0, head_tilt: 0.0}):
        ctx.expect_within(
            carriage,
            rail,
            axes="yz",
            inner_elem="runner_block",
            margin=0.0,
            name="runner block stays within the track channel envelope",
        )
        ctx.expect_contact(
            lamp_head,
            swivel_yoke,
            elem_a="left_trunnion",
            elem_b="left_cheek",
            name="left trunnion meets the left yoke cheek",
        )
        ctx.expect_contact(
            lamp_head,
            swivel_yoke,
            elem_a="right_trunnion",
            elem_b="right_cheek",
            name="right trunnion meets the right yoke cheek",
        )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({rail_slide: 0.460}):
        carriage_end = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            rail,
            axes="yz",
            inner_elem="runner_block",
            margin=0.0,
            name="runner block remains captured at full slide travel",
        )
        ctx.expect_overlap(
            carriage,
            rail,
            axes="x",
            elem_a="runner_block",
            min_overlap=0.090,
            name="runner block retains insertion in the rail at full travel",
        )
    ctx.check(
        "carriage slides along the rail axis",
        carriage_rest is not None
        and carriage_end is not None
        and carriage_end[0] > carriage_rest[0] + 0.40
        and abs(carriage_end[1] - carriage_rest[1]) < 1e-6
        and abs(carriage_end[2] - carriage_rest[2]) < 1e-6,
        details=f"rest={carriage_rest}, end={carriage_end}",
    )

    with ctx.pose({rail_slide: 0.0, carriage_swivel: 0.0, head_tilt: 0.0}):
        neutral_bezel = elem_center(lamp_head, "front_bezel")
    with ctx.pose({rail_slide: 0.0, carriage_swivel: math.pi / 2.0, head_tilt: 0.0}):
        swiveled_bezel = elem_center(lamp_head, "front_bezel")
    ctx.check(
        "swivel rolls the spot about the rail axis",
        neutral_bezel is not None
        and swiveled_bezel is not None
        and abs(swiveled_bezel[1] - neutral_bezel[1]) > 0.15
        and abs(swiveled_bezel[2] - neutral_bezel[2]) > 0.15,
        details=f"neutral={neutral_bezel}, swiveled={swiveled_bezel}",
    )

    with ctx.pose({rail_slide: 0.0, carriage_swivel: 0.0, head_tilt: 0.0}):
        neutral_glass = elem_center(lamp_head, "front_glass")
    with ctx.pose({rail_slide: 0.0, carriage_swivel: 0.0, head_tilt: 0.80}):
        tilted_glass = elem_center(lamp_head, "front_glass")
    ctx.check(
        "tilt knuckle pitches the lamp forward",
        neutral_glass is not None
        and tilted_glass is not None
        and tilted_glass[0] > neutral_glass[0] + 0.08
        and tilted_glass[2] > neutral_glass[2] + 0.04,
        details=f"neutral={neutral_glass}, tilted={tilted_glass}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
