from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    TestContext,
    TestReport,
)


def _dish_shell_mesh():
    """Thin revolved shell, rotated so its optical axis is local +X."""

    shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.045, -0.325),
            (0.200, -0.270),
            (0.440, -0.150),
            (0.640, -0.040),
            (0.760, 0.018),
        ],
        inner_profile=[
            (0.025, -0.275),
            (0.175, -0.225),
            (0.390, -0.120),
            (0.600, -0.020),
            (0.705, 0.002),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )
    shell.rotate_y(math.pi / 2.0).translate(0.485, 0.0, 0.0)
    return mesh_from_geometry(shell, "dish_shell")


def _rim_mesh():
    rim = TorusGeometry(radius=0.735, tube=0.020, radial_segments=24, tubular_segments=96)
    rim.rotate_y(math.pi / 2.0).translate(0.505, 0.0, 0.0)
    return mesh_from_geometry(rim, "rim_ring")


def _rear_rib_mesh(angle: float, name: str):
    c = math.cos(angle)
    s = math.sin(angle)
    rib = tube_from_spline_points(
        [
            (0.205, 0.105 * c, 0.105 * s),
            (0.335, 0.410 * c, 0.410 * s),
            (0.498, 0.718 * c, 0.718 * s),
        ],
        radius=0.012,
        samples_per_segment=7,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(rib, name)


def _feed_strut_mesh(angle: float, name: str):
    c = math.cos(angle)
    s = math.sin(angle)
    strut = tube_from_spline_points(
        [
            (0.505, 0.725 * c, 0.725 * s),
            (0.725, 0.330 * c, 0.330 * s),
            (1.000, 0.086 * c, 0.086 * s),
        ],
        radius=0.012,
        samples_per_segment=7,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(strut, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_radar_dish")

    paint = model.material("matte_olive", rgba=(0.28, 0.34, 0.24, 1.0))
    dark = model.material("gunmetal", rgba=(0.09, 0.10, 0.10, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    pale = model.material("pale_composite", rgba=(0.72, 0.75, 0.68, 1.0))
    amber = model.material("amber_lens", rgba=(1.0, 0.60, 0.12, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.55, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark,
        name="anchor_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.22, length=1.05),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=paint,
        name="main_column",
    )
    pedestal.visual(
        Cylinder(radius=0.34, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 1.195)),
        material=dark,
        name="fixed_bearing",
    )
    for i, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        x = 0.175 * math.cos(angle)
        y = 0.175 * math.sin(angle)
        yaw = angle
        pedestal.visual(
            Box((0.095, 0.36, 0.72)),
            origin=Origin(xyz=(x, y, 0.47), rpy=(0.0, 0.0, yaw)),
            material=paint,
            name=f"column_gusset_{i}",
        )
    pedestal.visual(
        Box((0.34, 0.22, 0.50)),
        origin=Origin(xyz=(-0.58, 0.0, 0.35)),
        material=dark,
        name="service_cabinet",
    )
    pedestal.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.24, 0.32),
                0.010,
                slot_size=(0.075, 0.010),
                pitch=(0.095, 0.045),
                frame=0.024,
                corner_radius=0.006,
                slot_angle_deg=0.0,
            ),
            "cabinet_vent",
        ),
        origin=Origin(xyz=(-0.753, 0.0, 0.40), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="cabinet_vent",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.39, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark,
        name="rotating_bearing",
    )
    turntable.visual(
        Box((1.05, 0.92, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=paint,
        name="equipment_deck",
    )
    turntable.visual(
        Box((0.48, 0.12, 0.95)),
        origin=Origin(xyz=(0.06, 0.52, 0.575)),
        material=paint,
        name="yoke_arm_0",
    )
    turntable.visual(
        Box((0.48, 0.12, 0.95)),
        origin=Origin(xyz=(0.06, -0.52, 0.575)),
        material=paint,
        name="yoke_arm_1",
    )
    turntable.visual(
        Cylinder(radius=0.145, length=0.065),
        origin=Origin(xyz=(0.0, 0.61, 0.98), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="bearing_boss_0",
    )
    turntable.visual(
        Cylinder(radius=0.145, length=0.065),
        origin=Origin(xyz=(0.0, -0.61, 0.98), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="bearing_boss_1",
    )
    turntable.visual(
        Box((0.38, 0.54, 0.40)),
        origin=Origin(xyz=(-0.47, 0.0, 0.35)),
        material=dark,
        name="receiver_rack",
    )
    turntable.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.34, 0.26),
                0.010,
                slot_size=(0.090, 0.010),
                pitch=(0.112, 0.040),
                frame=0.026,
                corner_radius=0.006,
                stagger=True,
            ),
            "rack_vent",
        ),
        origin=Origin(xyz=(-0.656, 0.0, 0.38), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rack_vent",
    )
    turntable.visual(
        Cylinder(radius=0.030, length=0.23),
        origin=Origin(xyz=(-0.45, -0.20, 0.66)),
        material=dark,
        name="warning_beacon_stem",
    )
    turntable.visual(
        Cylinder(radius=0.055, length=0.055),
        origin=Origin(xyz=(-0.45, -0.20, 0.80)),
        material=amber,
        name="warning_beacon",
    )

    antenna = model.part("antenna")
    antenna.visual(_dish_shell_mesh(), material=pale, name="dish_shell")
    antenna.visual(_rim_mesh(), material=paint, name="rim_ring")
    for i, angle in enumerate([j * math.tau / 8.0 for j in range(8)]):
        antenna.visual(_rear_rib_mesh(angle, f"rear_rib_{i}"), material=paint, name=f"rear_rib_{i}")
    antenna.visual(
        Cylinder(radius=0.135, length=0.30),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="central_hub",
    )
    antenna.visual(
        Cylinder(radius=0.080, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="trunnion",
    )
    antenna.visual(
        Box((0.26, 0.30, 0.22)),
        origin=Origin(xyz=(-0.22, 0.0, 0.0)),
        material=dark,
        name="counterweight",
    )
    antenna.visual(
        Box((0.25, 0.28, 0.18)),
        origin=Origin(xyz=(0.23, -0.28, -0.23), rpy=(0.0, 0.0, -0.20)),
        material=dark,
        name="waveguide_box",
    )
    for i, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        antenna.visual(_feed_strut_mesh(angle, f"feed_strut_{i}"), material=dark, name=f"feed_strut_{i}")
    feed_ring = TorusGeometry(radius=0.083, tube=0.012, radial_segments=14, tubular_segments=48)
    feed_ring.rotate_y(math.pi / 2.0).translate(0.985, 0.0, 0.0)
    antenna.visual(mesh_from_geometry(feed_ring, "feed_ring"), material=dark, name="feed_ring")
    antenna.visual(
        Cylinder(radius=0.078, length=0.23),
        origin=Origin(xyz=(1.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="feed_horn",
    )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=antenna,
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.35, lower=-0.18, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    turntable = object_model.get_part("turntable")
    antenna = object_model.get_part("antenna")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    def element_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    for cheek, boss in (("yoke_arm_0", "bearing_boss_0"), ("yoke_arm_1", "bearing_boss_1")):
        ctx.allow_overlap(
            turntable,
            antenna,
            elem_a=cheek,
            elem_b="trunnion",
            reason="The trunnion shaft is intentionally captured in the yoke cheek bearing bore.",
        )
        ctx.allow_overlap(
            turntable,
            antenna,
            elem_a=boss,
            elem_b="trunnion",
            reason="The exterior bearing boss is a simplified solid bushing around the captured trunnion.",
        )
        ctx.expect_within(
            antenna,
            turntable,
            axes="xz",
            inner_elem="trunnion",
            outer_elem=cheek,
            margin=0.02,
            name=f"trunnion centered in {cheek}",
        )
        ctx.expect_overlap(
            antenna,
            turntable,
            axes="y",
            elem_a="trunnion",
            elem_b=cheek,
            min_overlap=0.050,
            name=f"trunnion retained by {cheek}",
        )

    ctx.expect_gap(
        turntable,
        "pedestal",
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rotating_bearing",
        negative_elem="fixed_bearing",
        name="azimuth bearing seats on pedestal",
    )

    rest_feed = element_center(antenna, "feed_horn")
    with ctx.pose({elevation: 0.85}):
        elevated_feed = element_center(antenna, "feed_horn")
        ctx.expect_overlap(
            antenna,
            turntable,
            axes="y",
            elem_a="trunnion",
            elem_b="yoke_arm_0",
            min_overlap=0.050,
            name="raised dish keeps trunnion captured",
        )

    with ctx.pose({azimuth: 1.0}):
        turned_feed = element_center(antenna, "feed_horn")

    ctx.check(
        "elevation raises optical axis",
        rest_feed is not None and elevated_feed is not None and elevated_feed[2] > rest_feed[2] + 0.45,
        details=f"rest={rest_feed}, elevated={elevated_feed}",
    )
    ctx.check(
        "azimuth joint rotates assembly",
        rest_feed is not None and turned_feed is not None and abs(turned_feed[1] - rest_feed[1]) > 0.55,
        details=f"rest={rest_feed}, turned={turned_feed}",
    )

    return ctx.report()


object_model = build_object_model()
