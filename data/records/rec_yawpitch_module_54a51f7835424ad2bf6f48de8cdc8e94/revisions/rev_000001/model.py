from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_origin(
    xyz: tuple[float, float, float],
    *,
    axis: str = "z",
    yaw: float = 0.0,
) -> Origin:
    """Return an origin that points a URDF-style cylinder along a named axis."""
    if axis == "x":
        return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, yaw))
    if axis == "y":
        return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, yaw))
    return Origin(xyz=xyz, rpy=(0.0, 0.0, yaw))


def _add_radial_box(
    part,
    *,
    name: str,
    radius: float,
    angle: float,
    z: float,
    size: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(
            xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
            rpy=(0.0, 0.0, angle),
        ),
        material=material,
        name=name,
    )


def _add_bolt_circle(
    part,
    *,
    prefix: str,
    radius: float,
    z: float,
    count: int,
    head_radius: float,
    head_height: float,
    material,
    angle_offset: float = 0.0,
) -> None:
    for i in range(count):
        angle = angle_offset + (2.0 * math.pi * i / count)
        part.visual(
            Cylinder(radius=head_radius, length=head_height),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z)
            ),
            material=material,
            name=f"{prefix}_{i}",
        )


def _add_yoke_side(
    part,
    *,
    side: int,
    material,
    dark_material,
    pitch_axis_z: float,
) -> None:
    """Build one fabricated side yoke as bars around a real open trunnion bore."""
    y = side * 0.168
    suffix = "0" if side < 0 else "1"

    part.visual(
        Box((0.160, 0.100, 0.032)),
        origin=Origin(xyz=(0.0, side * 0.125, 0.064)),
        material=material,
        name=f"yoke_foot_{suffix}",
    )
    part.visual(
        Box((0.145, 0.026, 0.190)),
        origin=Origin(xyz=(0.0, y, 0.150)),
        material=material,
        name=f"yoke_lower_{suffix}",
    )
    for x, label in [(-0.062, "side_a"), (0.062, "side_b")]:
        part.visual(
            Box((0.026, 0.026, 0.170)),
            origin=Origin(xyz=(x, y, pitch_axis_z)),
            material=material,
            name=f"yoke_{label}_{suffix}",
        )
    part.visual(
        Box((0.145, 0.026, 0.060)),
        origin=Origin(xyz=(0.0, y, pitch_axis_z + 0.088)),
        material=material,
        name=f"yoke_upper_{suffix}",
    )

    part.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.043, tube=0.009, radial_segments=18, tubular_segments=48),
            f"pitch_bearing_ring_{suffix}",
        ),
        origin=Origin(xyz=(0.0, y, pitch_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_material,
        name=f"bearing_ring_{suffix}",
    )
    part.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.064, tube=0.0035, radial_segments=12, tubular_segments=54),
            f"pitch_index_band_{suffix}",
        ),
        origin=Origin(
            xyz=(0.0, y - side * 0.010, pitch_axis_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_material,
        name=f"pitch_index_band_{suffix}",
    )

    # Raised tick blocks on the supported index-band quadrants.
    for j, deg in enumerate((-70, -45, -20, 20, 45, 70)):
        angle = math.radians(deg)
        x = 0.073 * math.sin(angle)
        z = pitch_axis_z + 0.073 * math.cos(angle)
        part.visual(
            Box((0.006, 0.007, 0.018)),
            origin=Origin(xyz=(x, y - side * 0.014, z), rpy=(0.0, angle, 0.0)),
            material=dark_material,
            name=f"pitch_tick_{suffix}_{j}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_mechanical_study")

    steel = model.material("blasted_steel", rgba=(0.48, 0.50, 0.50, 1.0))
    dark = model.material("black_oxide", rgba=(0.055, 0.058, 0.060, 1.0))
    machined = model.material("machined_faces", rgba=(0.70, 0.72, 0.70, 1.0))
    cover_mat = model.material("oiled_cover_plate", rgba=(0.23, 0.25, 0.26, 1.0))
    bearing = model.material("bearing_bronze", rgba=(0.79, 0.55, 0.28, 1.0))

    yaw_plane_z = 0.106
    pitch_axis_z = 0.300

    base = model.part("base")
    base.visual(
        Box((0.620, 0.480, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark,
        name="ground_plate",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0565)),
        material=steel,
        name="welded_pedestal",
    )
    base.visual(
        Cylinder(radius=0.240, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=steel,
        name="outer_bolt_flange",
    )
    base.visual(
        Cylinder(radius=0.176, length=0.021),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=machined,
        name="lower_yaw_race",
    )
    base.visual(
        Cylinder(radius=0.154, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=machined,
        name="upper_yaw_race",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.174, tube=0.004, radial_segments=16, tubular_segments=64),
            "fixed_yaw_index_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=bearing,
        name="fixed_index_band",
    )
    for i, (x, y) in enumerate(
        [(-0.260, -0.190), (-0.260, 0.190), (0.260, -0.190), (0.260, 0.190)]
    ):
        base.visual(
            Cylinder(radius=0.025, length=0.007),
            origin=Origin(xyz=(x, y, 0.0375)),
            material=machined,
            name=f"anchor_washer_{i}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.009),
            origin=Origin(xyz=(x, y, 0.044)),
            material=dark,
            name=f"anchor_bolt_{i}",
        )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _add_radial_box(
            base,
            name=f"pedestal_gusset_{i}",
            radius=0.205,
            angle=angle,
            z=0.054,
            size=(0.190, 0.026, 0.040),
            material=steel,
        )
    for i, (x, y) in enumerate(((0.252, -0.092), (0.252, 0.092))):
        base.visual(
            Box((0.036, 0.050, 0.045)),
            origin=Origin(xyz=(x, y, 0.0575)),
            material=steel,
            name=f"yaw_stop_block_{i}",
        )
        base.visual(
            Cylinder(radius=0.011, length=0.009),
            origin=Origin(xyz=(x, y, 0.0845)),
            material=dark,
            name=f"stop_cap_screw_{i}",
        )
    _add_bolt_circle(
        base,
        prefix="race_socket_screw",
        radius=0.222,
        z=0.097,
        count=12,
        head_radius=0.008,
        head_height=0.008,
        material=dark,
        angle_offset=math.radians(15.0),
    )

    yaw_carrier = model.part("yaw_carrier")
    yaw_carrier.visual(
        Cylinder(radius=0.162, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=machined,
        name="yaw_turntable",
    )
    yaw_carrier.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.150, tube=0.0035, radial_segments=12, tubular_segments=72),
            "rotating_yaw_index_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=bearing,
        name="rotating_index_band",
    )
    for i in range(48):
        angle = (2.0 * math.pi * i) / 48.0
        length = 0.018 if i % 6 == 0 else 0.010
        width = 0.0038 if i % 6 == 0 else 0.0028
        _add_radial_box(
            yaw_carrier,
            name=f"yaw_scale_tick_{i}",
            radius=0.153,
            angle=angle,
            z=0.032,
            size=(length, width, 0.004),
            material=dark,
        )
    yaw_carrier.visual(
        Box((0.430, 0.198, 0.039)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=steel,
        name="rotating_saddle",
    )
    yaw_carrier.visual(
        Box((0.185, 0.108, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=cover_mat,
        name="yaw_access_cover",
    )
    for i, (x, y) in enumerate(
        [(-0.075, -0.040), (-0.075, 0.040), (0.075, -0.040), (0.075, 0.040)]
    ):
        yaw_carrier.visual(
            Cylinder(radius=0.0065, length=0.005),
            origin=Origin(xyz=(x, y, 0.0705)),
            material=dark,
            name=f"yaw_cover_screw_{i}",
        )
    _add_bolt_circle(
        yaw_carrier,
        prefix="turntable_socket_screw",
        radius=0.122,
        z=0.027,
        count=10,
        head_radius=0.007,
        head_height=0.006,
        material=dark,
        angle_offset=math.radians(18.0),
    )
    yaw_carrier.visual(
        Box((0.032, 0.070, 0.052)),
        origin=Origin(xyz=(0.205, 0.0, 0.052)),
        material=steel,
        name="yaw_stop_lug",
    )
    yaw_carrier.visual(
        Box((0.070, 0.034, 0.085)),
        origin=Origin(xyz=(-0.160, 0.0, 0.100)),
        material=steel,
        name="rear_service_bracket",
    )
    yaw_carrier.visual(
        Cylinder(radius=0.011, length=0.035),
        origin=_cylinder_origin((-0.160, 0.0, 0.148), axis="y"),
        material=dark,
        name="service_cross_pin",
    )
    _add_yoke_side(
        yaw_carrier,
        side=-1,
        material=steel,
        dark_material=bearing,
        pitch_axis_z=pitch_axis_z,
    )
    _add_yoke_side(
        yaw_carrier,
        side=1,
        material=steel,
        dark_material=bearing,
        pitch_axis_z=pitch_axis_z,
    )
    for i, (x, y, z) in enumerate(
        [
            (-0.086, -0.143, pitch_axis_z - 0.050),
            (0.086, -0.143, pitch_axis_z - 0.050),
            (-0.092, 0.143, pitch_axis_z + 0.034),
            (0.092, 0.143, pitch_axis_z + 0.034),
        ]
    ):
        yaw_carrier.visual(
            Box((0.030, 0.026, 0.026)),
            origin=Origin(xyz=(x, y, z)),
            material=dark,
            name=f"pitch_stop_block_{i}",
        )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.016, length=0.410),
        origin=_cylinder_origin((0.0, 0.0, 0.0), axis="y"),
        material=machined,
        name="trunnion_shaft",
    )
    for i, y in enumerate((-0.168, 0.168)):
        pitch_cradle.visual(
            Cylinder(radius=0.0345, length=0.018),
            origin=_cylinder_origin((0.0, y, 0.0), axis="y"),
            material=machined,
            name=f"bearing_journal_{i}",
        )
    pitch_cradle.visual(
        Cylinder(radius=0.038, length=0.128),
        origin=_cylinder_origin((0.0, 0.0, 0.0), axis="y"),
        material=machined,
        name="central_hub",
    )
    for i, y in enumerate((-0.132, 0.132)):
        pitch_cradle.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=_cylinder_origin((0.0, y, 0.0), axis="y"),
            material=dark,
            name=f"shaft_collar_{i}",
        )
    pitch_cradle.visual(
        Box((0.230, 0.124, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=steel,
        name="cradle_top_rail",
    )
    for i, x in enumerate((-0.102, 0.102)):
        pitch_cradle.visual(
            Box((0.026, 0.124, 0.172)),
            origin=Origin(xyz=(x, 0.0, -0.108)),
            material=steel,
            name=f"cradle_side_rail_{i}",
        )
    pitch_cradle.visual(
        Box((0.230, 0.124, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
        material=steel,
        name="cradle_bottom_rail",
    )
    for i, x in enumerate((-0.062, 0.062)):
        pitch_cradle.visual(
            Box((0.030, 0.095, 0.080)),
            origin=Origin(xyz=(x, 0.0, -0.012)),
            material=steel,
            name=f"hub_web_{i}",
        )
    pitch_cradle.visual(
        Box((0.006, 0.104, 0.112)),
        origin=Origin(xyz=(0.112, 0.0, -0.112)),
        material=cover_mat,
        name="front_access_cover",
    )
    pitch_cradle.visual(
        Box((0.006, 0.104, 0.112)),
        origin=Origin(xyz=(-0.112, 0.0, -0.112)),
        material=cover_mat,
        name="rear_access_cover",
    )
    for i, (x, y, z) in enumerate(
        [
            (0.115, -0.040, -0.073),
            (0.115, 0.040, -0.073),
            (0.115, -0.040, -0.151),
            (0.115, 0.040, -0.151),
            (-0.115, -0.040, -0.073),
            (-0.115, 0.040, -0.073),
            (-0.115, -0.040, -0.151),
            (-0.115, 0.040, -0.151),
        ]
    ):
        pitch_cradle.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=_cylinder_origin((x, y, z), axis="x"),
            material=dark,
            name=f"cover_screw_{i}",
        )
    pitch_cradle.visual(
        Box((0.036, 0.028, 0.046)),
        origin=Origin(xyz=(0.013, 0.130, 0.000)),
        material=dark,
        name="pitch_pointer",
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_carrier,
        origin=Origin(xyz=(0.0, 0.0, yaw_plane_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-1.20, upper=1.20),
        motion_properties=MotionProperties(damping=0.18, friction=0.04),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_carrier,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, pitch_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=-0.70, upper=0.75),
        motion_properties=MotionProperties(damping=0.12, friction=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yaw = object_model.get_part("yaw_carrier")
    pitch = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("base_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.check(
        "two explicit revolute axes",
        yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and pitch_joint.articulation_type == ArticulationType.REVOLUTE
        and yaw_joint.axis == (0.0, 0.0, 1.0)
        and pitch_joint.axis == (0.0, 1.0, 0.0),
        details=f"yaw={yaw_joint.articulation_type}/{yaw_joint.axis}, pitch={pitch_joint.articulation_type}/{pitch_joint.axis}",
    )
    ctx.expect_gap(
        yaw,
        base,
        axis="z",
        positive_elem="yaw_turntable",
        negative_elem="upper_yaw_race",
        max_gap=0.0005,
        max_penetration=0.0001,
        name="yaw turntable seats on lower race",
    )
    ctx.expect_within(
        pitch,
        yaw,
        axes="xz",
        inner_elem="trunnion_shaft",
        margin=0.010,
        name="trunnion shaft sits inside supported yoke span",
    )
    ctx.expect_overlap(
        pitch,
        yaw,
        axes="y",
        elem_a="trunnion_shaft",
        min_overlap=0.080,
        name="trunnion shaft spans both side yokes",
    )
    for i in (0, 1):
        ctx.allow_overlap(
            pitch,
            yaw,
            elem_a=f"bearing_journal_{i}",
            elem_b=f"bearing_ring_{i}",
            reason="The pitch journal is intentionally seated in the visible trunnion bearing ring as a captured running fit.",
        )
        ctx.expect_within(
            pitch,
            yaw,
            axes="xz",
            inner_elem=f"bearing_journal_{i}",
            outer_elem=f"bearing_ring_{i}",
            margin=0.0,
            name=f"pitch journal {i} is concentric with bearing ring",
        )
        ctx.expect_overlap(
            pitch,
            yaw,
            axes="y",
            elem_a=f"bearing_journal_{i}",
            elem_b=f"bearing_ring_{i}",
            min_overlap=0.010,
            name=f"pitch journal {i} remains captured in bearing",
        )

    def _center_from_aabb(aabb, index: int) -> float:
        return 0.5 * (aabb[0][index] + aabb[1][index])

    rest_lug = ctx.part_element_world_aabb(yaw, elem="yaw_stop_lug")
    rest_cover = ctx.part_element_world_aabb(pitch, elem="front_access_cover")
    with ctx.pose({yaw_joint: 0.65}):
        turned_lug = ctx.part_element_world_aabb(yaw, elem="yaw_stop_lug")
    with ctx.pose({pitch_joint: 0.55}):
        tipped_cover = ctx.part_element_world_aabb(pitch, elem="front_access_cover")

    ctx.check(
        "yaw stop lug sweeps with yaw axis",
        rest_lug is not None
        and turned_lug is not None
        and _center_from_aabb(turned_lug, 1) > _center_from_aabb(rest_lug, 1) + 0.10,
        details=f"rest={rest_lug}, yawed={turned_lug}",
    )
    ctx.check(
        "pitch cradle rotates about trunnions",
        rest_cover is not None
        and tipped_cover is not None
        and abs(_center_from_aabb(tipped_cover, 2) - _center_from_aabb(rest_cover, 2)) > 0.030,
        details=f"rest={rest_cover}, pitched={tipped_cover}",
    )

    return ctx.report()


object_model = build_object_model()
