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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


GLOBE_RADIUS = 0.105
MERIDIAN_RING_RADIUS = 0.127
MERIDIAN_RING_TUBE = 0.006
TILT_AXIS_HEIGHT = 0.305
MERIDIAN_REST_TILT = math.radians(23.5)
POLAR_GAP_ANGLE = math.radians(22.0)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_arc_points(radius: float, start_angle: float, end_angle: float, *, samples: int = 13):
    return [
        (
            0.0,
            radius * math.sin(start_angle + (end_angle - start_angle) * i / (samples - 1)),
            radius * math.cos(start_angle + (end_angle - start_angle) * i / (samples - 1)),
        )
        for i in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="antique_desk_globe")

    walnut = model.material("walnut", rgba=(0.30, 0.18, 0.10, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.22, 0.13, 0.07, 1.0))
    antique_brass = model.material("antique_brass", rgba=(0.69, 0.58, 0.31, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.57, 0.46, 0.24, 1.0))
    parchment = model.material("parchment", rgba=(0.84, 0.79, 0.64, 1.0))
    sepia = model.material("sepia", rgba=(0.42, 0.31, 0.18, 1.0))

    foot_geom = _mesh(
        "pedestal_foot",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.035),
                (0.050, 0.0, 0.032),
                (0.103, 0.0, 0.020),
                (0.145, 0.0, 0.012),
            ],
            radius=0.012,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    support_arm_pos_geom = _mesh(
        "support_arm_pos",
        tube_from_spline_points(
            [
                (0.0, 0.018, 0.176),
                (-0.022, 0.095, 0.179),
                (-0.055, 0.146, 0.185),
                (-0.090, 0.150, 0.245),
                (-0.090, 0.150, TILT_AXIS_HEIGHT),
            ],
            radius=0.007,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    support_arm_neg_geom = _mesh(
        "support_arm_neg",
        tube_from_spline_points(
            [
                (0.0, -0.018, 0.176),
                (-0.022, -0.095, 0.179),
                (-0.055, -0.146, 0.185),
                (-0.090, -0.150, 0.245),
                (-0.090, -0.150, TILT_AXIS_HEIGHT),
            ],
            radius=0.007,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    meridian_arc_pos_geom = _mesh(
        "meridian_arc_pos",
        tube_from_spline_points(
            _yz_arc_points(
                MERIDIAN_RING_RADIUS,
                POLAR_GAP_ANGLE,
                math.pi - POLAR_GAP_ANGLE,
                samples=15,
            ),
            radius=MERIDIAN_RING_TUBE,
            samples_per_segment=8,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    meridian_arc_neg_geom = _mesh(
        "meridian_arc_neg",
        tube_from_spline_points(
            _yz_arc_points(
                MERIDIAN_RING_RADIUS,
                -(math.pi - POLAR_GAP_ANGLE),
                -POLAR_GAP_ANGLE,
                samples=15,
            ),
            radius=MERIDIAN_RING_TUBE,
            samples_per_segment=8,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    longitude_band_geom = _mesh(
        "prime_meridian_band",
        TorusGeometry(radius=GLOBE_RADIUS - 0.001, tube=0.0012, radial_segments=10, tubular_segments=60),
    )

    base = model.part("base")
    base.visual(
        foot_geom,
        material=walnut,
        name="foot_pos_x",
    )
    base.visual(
        foot_geom,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=walnut,
        name="foot_pos_y",
    )
    base.visual(
        foot_geom,
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=walnut,
        name="foot_neg_x",
    )
    base.visual(
        foot_geom,
        origin=Origin(rpy=(0.0, 0.0, 3.0 * math.pi / 2.0)),
        material=walnut,
        name="foot_neg_y",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_walnut,
        name="pedestal_hub",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=walnut,
        name="pedestal_neck",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=walnut,
        name="center_post",
    )
    base.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=dark_walnut,
        name="top_finial",
    )
    base.visual(
        support_arm_pos_geom,
        material=aged_brass,
        name="support_arm_pos",
    )
    base.visual(
        support_arm_neg_geom,
        material=aged_brass,
        name="support_arm_neg",
    )
    base.visual(
        Cylinder(radius=0.0055, length=0.082),
        origin=Origin(xyz=(-0.051, 0.150, TILT_AXIS_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="support_connector_pos",
    )
    base.visual(
        Cylinder(radius=0.0055, length=0.082),
        origin=Origin(xyz=(-0.051, -0.150, TILT_AXIS_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="support_connector_neg",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.158, TILT_AXIS_HEIGHT), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=antique_brass,
        name="support_collar_pos",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, -0.158, TILT_AXIS_HEIGHT), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=antique_brass,
        name="support_collar_neg",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.32, 0.32, 0.20)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    meridian = model.part("meridian")
    meridian.visual(
        meridian_arc_pos_geom,
        material=antique_brass,
        name="meridian_arc_pos",
    )
    meridian.visual(
        meridian_arc_neg_geom,
        material=antique_brass,
        name="meridian_arc_neg",
    )
    meridian.visual(
        Cylinder(radius=0.0042, length=0.024),
        origin=Origin(xyz=(0.0, 0.140, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="trunnion_pos",
    )
    meridian.visual(
        Cylinder(radius=0.0042, length=0.024),
        origin=Origin(xyz=(0.0, -0.140, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="trunnion_neg",
    )
    meridian.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=aged_brass,
        name="upper_pivot_bearing",
    )
    meridian.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.122)),
        material=aged_brass,
        name="lower_pivot_bearing",
    )
    for name, xyz in [
        ("upper_bridge_pos_left", (0.0045, 0.024, 0.121)),
        ("upper_bridge_pos_right", (-0.0045, 0.024, 0.121)),
        ("upper_bridge_neg_left", (0.0045, -0.024, 0.121)),
        ("upper_bridge_neg_right", (-0.0045, -0.024, 0.121)),
        ("lower_bridge_pos_left", (0.0045, 0.024, -0.121)),
        ("lower_bridge_pos_right", (-0.0045, 0.024, -0.121)),
        ("lower_bridge_neg_left", (0.0045, -0.024, -0.121)),
        ("lower_bridge_neg_right", (-0.0045, -0.024, -0.121)),
    ]:
        meridian.visual(
            Cylinder(radius=0.0032, length=0.048),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aged_brass,
            name=name,
        )
    meridian.inertial = Inertial.from_geometry(
        Box((0.025, 0.320, 0.320)),
        mass=0.65,
        origin=Origin(),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=parchment,
        name="globe_sphere",
    )
    globe.visual(
        longitude_band_geom,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=sepia,
        name="prime_meridian_band",
    )
    globe.visual(
        Cylinder(radius=0.0038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + 0.006)),
        material=aged_brass,
        name="north_pivot",
    )
    globe.visual(
        Cylinder(radius=0.0038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -(GLOBE_RADIUS + 0.006))),
        material=aged_brass,
        name="south_pivot",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=GLOBE_RADIUS),
        mass=0.9,
    )

    model.articulation(
        "base_to_meridian",
        ArticulationType.REVOLUTE,
        parent=base,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, TILT_AXIS_HEIGHT), rpy=(0.0, MERIDIAN_REST_TILT, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    tilt_joint = object_model.get_articulation("base_to_meridian")
    spin_joint = object_model.get_articulation("meridian_to_globe")

    with ctx.pose({tilt_joint: 0.0, spin_joint: 0.0}):
        ctx.expect_within(
            meridian,
            base,
            axes="xz",
            inner_elem="trunnion_pos",
            outer_elem="support_collar_pos",
            name="positive trunnion sits inside positive support collar footprint",
        )
        ctx.expect_within(
            meridian,
            base,
            axes="xz",
            inner_elem="trunnion_neg",
            outer_elem="support_collar_neg",
            name="negative trunnion sits inside negative support collar footprint",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a="north_pivot",
            elem_b="upper_pivot_bearing",
            name="north pivot stays seated in contact with the upper bearing",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a="south_pivot",
            elem_b="lower_pivot_bearing",
            name="south pivot stays seated in contact with the lower bearing",
        )
        ctx.expect_gap(
            globe,
            base,
            axis="z",
            positive_elem="globe_sphere",
            negative_elem="top_finial",
            min_gap=0.018,
            name="globe sphere clears the top finial",
        )

    north_rest = None
    north_tilted = None
    with ctx.pose({tilt_joint: 0.0}):
        aabb = ctx.part_element_world_aabb(globe, elem="north_pivot")
        if aabb is not None:
            north_rest = (
                0.5 * (aabb[0][0] + aabb[1][0]),
                0.5 * (aabb[0][2] + aabb[1][2]),
            )
    with ctx.pose({tilt_joint: 0.30}):
        aabb = ctx.part_element_world_aabb(globe, elem="north_pivot")
        if aabb is not None:
            north_tilted = (
                0.5 * (aabb[0][0] + aabb[1][0]),
                0.5 * (aabb[0][2] + aabb[1][2]),
            )
    ctx.check(
        "meridian tilt swings the north pivot forward",
        north_rest is not None
        and north_tilted is not None
        and north_tilted[0] > north_rest[0] + 0.020
        and north_tilted[1] < north_rest[1] - 0.005,
        details=f"rest={north_rest}, tilted={north_tilted}",
    )

    meridian_band_rest = None
    meridian_band_quarter_turn = None
    with ctx.pose({tilt_joint: -MERIDIAN_REST_TILT, spin_joint: 0.0}):
        meridian_band_rest = ctx.part_element_world_aabb(globe, elem="prime_meridian_band")
    with ctx.pose({tilt_joint: -MERIDIAN_REST_TILT, spin_joint: math.pi / 2.0}):
        meridian_band_quarter_turn = ctx.part_element_world_aabb(globe, elem="prime_meridian_band")

    def _span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    rest_x = _span(meridian_band_rest, 0)
    rest_y = _span(meridian_band_rest, 1)
    quarter_x = _span(meridian_band_quarter_turn, 0)
    quarter_y = _span(meridian_band_quarter_turn, 1)
    ctx.check(
        "globe spin rotates the visible longitude band around the polar axis",
        rest_x is not None
        and rest_y is not None
        and quarter_x is not None
        and quarter_y is not None
        and rest_x > rest_y + 0.12
        and quarter_y > quarter_x + 0.12,
        details=(
            f"rest_spans=({rest_x}, {rest_y}), "
            f"quarter_turn_spans=({quarter_x}, {quarter_y})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
