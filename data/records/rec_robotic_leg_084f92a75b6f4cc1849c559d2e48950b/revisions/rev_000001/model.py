from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _cyl_y(xyz=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def _cyl_x(xyz=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _rounded_prism(width_x: float, width_y: float, height_z: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width_x, width_y, radius, corner_segments=8),
            height_z,
            center=True,
        ),
        name,
    )


def _tube(points, radius: float, name: str, *, segments: int = 18):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            radial_segments=segments,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_leg")

    matte_graphite = _mat("matte_graphite", (0.065, 0.070, 0.075, 1.0))
    satin_titanium = _mat("satin_titanium", (0.52, 0.54, 0.55, 1.0))
    bead_blast = _mat("bead_blast_aluminum", (0.70, 0.71, 0.69, 1.0))
    dark_steel = _mat("dark_steel", (0.18, 0.19, 0.20, 1.0))
    seam_black = _mat("seam_black", (0.015, 0.016, 0.018, 1.0))
    rubber = _mat("rubber", (0.025, 0.025, 0.023, 1.0))
    amber_lens = _mat("amber_lens", (0.95, 0.56, 0.13, 1.0))

    # Root frame: a compact pelvis-side hip yoke with an exposed pitch axis.
    hip_frame = model.part("hip_frame")
    hip_frame.visual(
        _rounded_prism(0.30, 0.28, 0.070, 0.030, "hip_top_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=bead_blast,
        name="top_plate",
    )
    hip_frame.visual(
        Box((0.18, 0.30, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=matte_graphite,
        name="upper_bridge",
    )
    for y in (-0.130, 0.130):
        hip_frame.visual(
            Box((0.135, 0.050, 0.180)),
            origin=Origin(xyz=(0.0, y, 0.000)),
            material=matte_graphite,
            name=f"hip_cheek_{0 if y < 0 else 1}",
        )
        hip_frame.visual(
            Cylinder(radius=0.092, length=0.046),
            origin=_cyl_y((0.0, y, 0.000)),
            material=satin_titanium,
            name=f"hip_outer_boss_{0 if y < 0 else 1}",
        )
        hip_frame.visual(
            Cylinder(radius=0.020, length=0.052),
            origin=_cyl_y((0.0, y, 0.000)),
            material=dark_steel,
            name=f"hip_axis_fastener_{0 if y < 0 else 1}",
        )
    hip_frame.visual(
        Box((0.245, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.136, 0.205)),
        material=seam_black,
        name="rear_seam",
    )
    hip_frame.visual(
        Box((0.245, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.136, 0.205)),
        material=seam_black,
        name="front_seam",
    )
    hip_frame.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.30)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    # Upper limb link: hip barrel, twin load rails, compact actuator bay, and
    # a knee clevis that leaves the child knee barrel visibly captured.
    upper_leg = model.part("upper_leg")
    upper_leg.visual(
        Cylinder(radius=0.074, length=0.210),
        origin=_cyl_y(),
        material=satin_titanium,
        name="hip_barrel",
    )
    upper_leg.visual(
        Cylinder(radius=0.027, length=0.178),
        origin=_cyl_y(),
        material=dark_steel,
        name="hip_axis_pin",
    )
    upper_leg.visual(
        Box((0.120, 0.135, 0.100)),
        origin=Origin(xyz=(0.018, 0.0, -0.103)),
        material=satin_titanium,
        name="hip_neck",
    )
    upper_leg.visual(
        _rounded_prism(0.138, 0.170, 0.305, 0.024, "upper_actuator_bay"),
        origin=Origin(xyz=(0.030, 0.0, -0.268)),
        material=matte_graphite,
        name="actuator_bay",
    )
    upper_leg.visual(
        Box((0.112, 0.126, 0.080)),
        origin=Origin(xyz=(0.016, 0.0, -0.406)),
        material=satin_titanium,
        name="knee_bridge",
    )
    upper_leg.visual(
        Box((0.095, 0.245, 0.060)),
        origin=Origin(xyz=(0.000, 0.0, -0.405)),
        material=matte_graphite,
        name="knee_top_yoke",
    )
    for cheek_name, boss_name, y in (
        ("knee_cheek_0", "knee_outer_boss_0", -0.118),
        ("knee_cheek_1", "knee_outer_boss_1", 0.118),
    ):
        upper_leg.visual(
            Box((0.122, 0.045, 0.172)),
            origin=Origin(xyz=(0.000, y, -0.520)),
            material=matte_graphite,
            name=cheek_name,
        )
        upper_leg.visual(
            Cylinder(radius=0.078, length=0.042),
            origin=_cyl_y((0.0, y, -0.520)),
            material=satin_titanium,
            name=boss_name,
        )
    for y in (-0.074, 0.074):
        upper_leg.visual(
            _tube(
                [
                    (-0.055, y, -0.032),
                    (-0.070, y, -0.205),
                    (-0.048, y, -0.385),
                    (-0.070, y, -0.418),
                ],
                0.020,
                f"upper_load_rail_{0 if y < 0 else 1}",
            ),
            material=bead_blast,
            name=f"load_rail_{0 if y < 0 else 1}",
        )
    upper_leg.visual(
        Box((0.006, 0.128, 0.245)),
        origin=Origin(xyz=(0.102, 0.0, -0.265)),
        material=seam_black,
        name="front_panel_gap",
    )
    upper_leg.visual(
        Box((0.014, 0.006, 0.238)),
        origin=Origin(xyz=(0.055, -0.088, -0.265)),
        material=seam_black,
        name="side_panel_gap_0",
    )
    upper_leg.visual(
        Box((0.014, 0.010, 0.238)),
        origin=Origin(xyz=(0.055, 0.086, -0.265)),
        material=seam_black,
        name="side_panel_gap_1",
    )
    for z in (-0.170, -0.360):
        for y in (-0.058, 0.058):
            upper_leg.visual(
                Cylinder(radius=0.010, length=0.007),
                origin=_cyl_x((0.105, y, z)),
                material=dark_steel,
                name=f"upper_screw_{len(upper_leg.visuals)}",
            )
    upper_leg.visual(
        Box((0.012, 0.055, 0.030)),
        origin=Origin(xyz=(0.101, 0.0, -0.118)),
        material=amber_lens,
        name="status_lens",
    )
    upper_leg.inertial = Inertial.from_geometry(
        Box((0.20, 0.25, 0.58)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
    )

    # Lower limb link: narrower shin member with visible knee trunnion and ankle fork.
    lower_leg = model.part("lower_leg")
    lower_leg.visual(
        Cylinder(radius=0.066, length=0.191),
        origin=_cyl_y(),
        material=satin_titanium,
        name="knee_barrel",
    )
    lower_leg.visual(
        Cylinder(radius=0.024, length=0.172),
        origin=_cyl_y(),
        material=dark_steel,
        name="knee_axis_pin",
    )
    lower_leg.visual(
        Box((0.098, 0.122, 0.090)),
        origin=Origin(xyz=(-0.008, 0.0, -0.085)),
        material=satin_titanium,
        name="knee_neck",
    )
    lower_leg.visual(
        _rounded_prism(0.112, 0.150, 0.265, 0.020, "lower_actuator_bay"),
        origin=Origin(xyz=(-0.026, 0.0, -0.223)),
        material=matte_graphite,
        name="shin_actuator_bay",
    )
    lower_leg.visual(
        Box((0.090, 0.110, 0.074)),
        origin=Origin(xyz=(-0.010, 0.0, -0.335)),
        material=satin_titanium,
        name="ankle_bridge",
    )
    lower_leg.visual(
        Box((0.080, 0.215, 0.050)),
        origin=Origin(xyz=(0.000, 0.0, -0.350)),
        material=matte_graphite,
        name="ankle_top_yoke",
    )
    for cheek_name, boss_name, y in (
        ("ankle_cheek_0", "ankle_outer_boss_0", -0.104),
        ("ankle_cheek_1", "ankle_outer_boss_1", 0.104),
    ):
        lower_leg.visual(
            Box((0.105, 0.040, 0.118)),
            origin=Origin(xyz=(0.000, y, -0.430)),
            material=matte_graphite,
            name=cheek_name,
        )
        lower_leg.visual(
            Cylinder(radius=0.061, length=0.038),
            origin=_cyl_y((0.0, y, -0.430)),
            material=satin_titanium,
            name=boss_name,
        )
    for y in (-0.060, 0.060):
        lower_leg.visual(
            _tube(
                [
                    (0.052, y, -0.030),
                    (0.068, y, -0.160),
                    (0.040, y, -0.315),
                    (0.060, y, -0.345),
                ],
                0.017,
                f"lower_load_rail_{0 if y < 0 else 1}",
                segments=16,
            ),
            material=bead_blast,
            name=f"shin_load_rail_{0 if y < 0 else 1}",
        )
    lower_leg.visual(
        Box((0.006, 0.108, 0.205)),
        origin=Origin(xyz=(-0.084, 0.0, -0.220)),
        material=seam_black,
        name="calf_panel_gap",
    )
    lower_leg.visual(
        Box((0.011, 0.006, 0.205)),
        origin=Origin(xyz=(-0.052, -0.078, -0.220)),
        material=seam_black,
        name="calf_side_gap_0",
    )
    lower_leg.visual(
        Box((0.011, 0.010, 0.205)),
        origin=Origin(xyz=(-0.052, 0.076, -0.220)),
        material=seam_black,
        name="calf_side_gap_1",
    )
    for z in (-0.130, -0.305):
        for y in (-0.048, 0.048):
            lower_leg.visual(
                Cylinder(radius=0.0085, length=0.006),
                origin=_cyl_x((-0.087, y, z)),
                material=dark_steel,
                name=f"lower_screw_{len(lower_leg.visuals)}",
            )
    lower_leg.inertial = Inertial.from_geometry(
        Box((0.17, 0.22, 0.50)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
    )

    # Foot link: stiff ankle trunnion, compact heel block, satin top plate, and
    # a low-profile rubberized sole sized like a real robotic/prosthetic foot.
    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.053, length=0.168),
        origin=_cyl_y(),
        material=satin_titanium,
        name="ankle_barrel",
    )
    foot.visual(
        Cylinder(radius=0.020, length=0.150),
        origin=_cyl_y(),
        material=dark_steel,
        name="ankle_axis_pin",
    )
    foot.visual(
        Box((0.075, 0.124, 0.090)),
        origin=Origin(xyz=(0.012, 0.0, -0.062)),
        material=matte_graphite,
        name="heel_upright",
    )
    foot.visual(
        _rounded_prism(0.355, 0.176, 0.042, 0.050, "foot_top_plate"),
        origin=Origin(xyz=(0.132, 0.0, -0.108)),
        material=bead_blast,
        name="top_plate",
    )
    foot.visual(
        _rounded_prism(0.385, 0.194, 0.034, 0.055, "foot_rubber_sole"),
        origin=Origin(xyz=(0.148, 0.0, -0.145)),
        material=rubber,
        name="rubber_sole",
    )
    foot.visual(
        Box((0.070, 0.150, 0.032)),
        origin=Origin(xyz=(-0.035, 0.0, -0.122)),
        material=dark_steel,
        name="heel_block",
    )
    foot.visual(
        Box((0.008, 0.156, 0.018)),
        origin=Origin(xyz=(0.003, 0.0, -0.084)),
        material=seam_black,
        name="ankle_seam",
    )
    for x in (0.040, 0.190, 0.300):
        foot.visual(
            Box((0.005, 0.168, 0.006)),
            origin=Origin(xyz=(x, 0.0, -0.162)),
            material=seam_black,
            name=f"sole_groove_{len(foot.visuals)}",
        )
    for y in (-0.064, 0.064):
        foot.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=_cyl_x((0.032, y, -0.062)),
            material=dark_steel,
            name=f"foot_screw_{0 if y < 0 else 1}",
        )
    foot.inertial = Inertial.from_geometry(
        Box((0.42, 0.22, 0.18)),
        mass=4.2,
        origin=Origin(xyz=(0.13, 0.0, -0.10)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_frame,
        child=upper_leg,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.8, lower=-0.65, upper=0.95),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg,
        child=lower_leg,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=170.0, velocity=3.2, lower=0.0, upper=1.85),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_leg,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=3.0, lower=-0.62, upper=0.68),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hip_frame = object_model.get_part("hip_frame")
    upper_leg = object_model.get_part("upper_leg")
    lower_leg = object_model.get_part("lower_leg")
    foot = object_model.get_part("foot")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    ctx.check(
        "serial hip knee ankle joints are present",
        all(j is not None for j in (hip_pitch, knee_pitch, ankle_pitch)),
        details="The leg should expose hip, knee, and ankle pitch joints.",
    )
    ctx.expect_origin_gap(
        hip_frame,
        lower_leg,
        axis="z",
        min_gap=0.49,
        max_gap=0.55,
        name="hip axis to knee axis spacing",
    )
    ctx.expect_origin_gap(
        lower_leg,
        foot,
        axis="z",
        min_gap=0.40,
        max_gap=0.46,
        name="knee axis to ankle axis spacing",
    )
    ctx.expect_overlap(
        upper_leg,
        lower_leg,
        axes="xz",
        min_overlap=0.06,
        elem_a="knee_cheek_0",
        elem_b="knee_barrel",
        name="knee barrel sits inside visible clevis zone",
    )
    ctx.expect_overlap(
        lower_leg,
        foot,
        axes="xz",
        min_overlap=0.045,
        elem_a="ankle_cheek_0",
        elem_b="ankle_barrel",
        name="ankle barrel sits inside visible fork zone",
    )

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({knee_pitch: 0.90}):
        flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "knee pitch flexes the distal leg backward",
        rest_foot is not None
        and flexed_foot is not None
        and flexed_foot[0] < rest_foot[0] - 0.15,
        details=f"rest_foot={rest_foot}, flexed_foot={flexed_foot}",
    )

    rest_lower = ctx.part_world_position(lower_leg)
    with ctx.pose({hip_pitch: 0.45}):
        swung_lower = ctx.part_world_position(lower_leg)
    ctx.check(
        "hip pitch moves the knee axis around the visible hip barrel",
        rest_lower is not None
        and swung_lower is not None
        and swung_lower[0] < rest_lower[0] - 0.15,
        details=f"rest_lower={rest_lower}, swung_lower={swung_lower}",
    )

    rest_aabb = ctx.part_element_world_aabb(foot, elem="rubber_sole")
    with ctx.pose({ankle_pitch: -0.62}):
        lifted_aabb = ctx.part_element_world_aabb(foot, elem="rubber_sole")
    ctx.check(
        "ankle pitch visibly lifts the forefoot",
        rest_aabb is not None
        and lifted_aabb is not None
        and lifted_aabb[1][2] > rest_aabb[1][2] + 0.08,
        details=f"rest_aabb={rest_aabb}, lifted_aabb={lifted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
