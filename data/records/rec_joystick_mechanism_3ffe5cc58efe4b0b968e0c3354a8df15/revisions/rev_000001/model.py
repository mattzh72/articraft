from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


CONTROL_Z = 0.055
OUTER_RING_RADIUS = 0.052
OUTER_TUBE_RADIUS = 0.0055
INNER_PIN_RADIUS = 0.0042
OUTER_PIN_RADIUS = 0.0048


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_console_joystick")

    dark_panel = _mat("satin_black_panel", (0.015, 0.016, 0.018, 1.0))
    cup_coating = _mat("charcoal_powdercoat", (0.055, 0.060, 0.065, 1.0))
    brushed_metal = _mat("brushed_gunmetal", (0.34, 0.35, 0.36, 1.0))
    black_rubber = _mat("black_rubber", (0.010, 0.010, 0.011, 1.0))
    screw_steel = _mat("dark_screw_steel", (0.12, 0.12, 0.13, 1.0))

    for material in (dark_panel, cup_coating, brushed_metal, black_rubber, screw_steel):
        model.materials.append(material)

    base = model.part("base")

    # A low panel-mount cup: thin hollow shell with a rolled lip and visible
    # countersunk fasteners on the console flange.
    base.visual(
        Cylinder(radius=0.132, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=dark_panel,
        name="panel_flange",
    )
    cup_shell = LatheGeometry.from_shell_profiles(
        [
            (0.035, 0.000),
            (0.088, 0.004),
            (0.106, 0.015),
            (0.112, 0.027),
        ],
        [
            (0.020, 0.005),
            (0.063, 0.007),
            (0.088, 0.017),
            (0.099, 0.030),
        ],
        segments=96,
        end_cap="round",
        lip_samples=10,
    )
    base.visual(
        mesh_from_geometry(cup_shell, "shallow_base_cup"),
        material=cup_coating,
        name="cup",
    )
    base.visual(
        mesh_from_geometry(
            TorusGeometry(0.101, 0.0045, radial_segments=12, tubular_segments=96),
            "rolled_lip",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cup_coating,
        name="rolled_lip",
    )
    # Side bearing collars for the outer left-right gimbal axis.  Each collar is
    # held above the cup by a short pedestal and a radial web so the pivot path
    # reads as physically supported rather than floating.
    for sign, suffix in ((1.0, "pos"), (-1.0, "neg")):
        base.visual(
            mesh_from_geometry(
                TorusGeometry(0.0070, 0.0024, radial_segments=10, tubular_segments=40),
                f"side_bearing_{suffix}",
            ),
            origin=Origin(xyz=(sign * 0.088, 0.0, CONTROL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"side_bearing_{suffix}",
        )
        base.visual(
            Box((0.018, 0.023, 0.034)),
            origin=Origin(xyz=(sign * 0.088, 0.0, CONTROL_Z - 0.024)),
            material=cup_coating,
            name=f"bearing_pedestal_{suffix}",
        )
        base.visual(
            Box((0.030, 0.014, 0.010)),
            origin=Origin(xyz=(sign * 0.073, 0.0, 0.035)),
            material=cup_coating,
            name=f"bearing_web_{suffix}",
        )

    for i, angle in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
        base.visual(
            Cylinder(radius=0.0055, length=0.0022),
            origin=Origin(xyz=(0.122 * math.cos(angle), 0.122 * math.sin(angle), 0.0041)),
            material=screw_steel,
            name=f"screw_{i}",
        )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_geometry(
            TorusGeometry(OUTER_RING_RADIUS, OUTER_TUBE_RADIUS, radial_segments=16, tubular_segments=96),
            "outer_hoop",
        ),
        material=brushed_metal,
        name="outer_hoop",
    )
    for sign, suffix in ((1.0, "pos"), (-1.0, "neg")):
        outer_ring.visual(
            Cylinder(radius=OUTER_PIN_RADIUS, length=0.072),
            origin=Origin(xyz=(sign * 0.071, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"side_pin_{suffix}",
        )
        outer_ring.visual(
            mesh_from_geometry(
                TorusGeometry(0.0078, 0.0020, radial_segments=10, tubular_segments=36),
                f"side_boss_{suffix}",
            ),
            origin=Origin(xyz=(sign * OUTER_RING_RADIUS, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"side_boss_{suffix}",
        )
    for sign, suffix in ((1.0, "front"), (-1.0, "rear")):
        outer_ring.visual(
            mesh_from_geometry(
                TorusGeometry(0.0083, 0.0021, radial_segments=10, tubular_segments=36),
                f"{suffix}_bearing",
            ),
            origin=Origin(xyz=(0.0, sign * OUTER_RING_RADIUS, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"{suffix}_bearing",
        )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.010, length=0.069),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="center_sleeve",
    )
    cradle.visual(
        Sphere(radius=0.012),
        material=brushed_metal,
        name="center_ball",
    )
    for sign, suffix in ((1.0, "front"), (-1.0, "rear")):
        cradle.visual(
            Cylinder(radius=INNER_PIN_RADIUS, length=0.040),
            origin=Origin(xyz=(0.0, sign * 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"{suffix}_pin",
        )
        cradle.visual(
            Box((0.013, 0.026, 0.007)),
            origin=Origin(xyz=(0.0, sign * 0.026, 0.006)),
            material=brushed_metal,
            name=f"{suffix}_cheek",
        )

    stick = model.part("stick")
    stick.visual(
        Cylinder(radius=0.0072, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=black_rubber,
        name="stem",
    )
    collar_geom = KnobGeometry(
        0.031,
        0.018,
        body_style="cylindrical",
        edge_radius=0.0008,
        grip=KnobGrip(style="knurled", count=36, depth=0.0010, helix_angle_deg=24.0),
    )
    stick.visual(
        mesh_from_geometry(collar_geom, "knurled_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=brushed_metal,
        name="collar",
    )
    stick.visual(
        Sphere(radius=0.0085),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=black_rubber,
        name="top_cap",
    )

    model.articulation(
        "base_to_outer_ring",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, CONTROL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-0.38, upper=0.38),
    )
    model.articulation(
        "outer_ring_to_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.38, upper=0.38),
    )
    model.articulation(
        "cradle_to_stick",
        ArticulationType.FIXED,
        parent=cradle,
        child=stick,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    outer_ring = object_model.get_part("outer_ring")
    cradle = object_model.get_part("cradle")
    stick = object_model.get_part("stick")
    outer_joint = object_model.get_articulation("base_to_outer_ring")
    inner_joint = object_model.get_articulation("outer_ring_to_cradle")

    # The inner trunnion pins are represented as captured in holes through the
    # solid outer hoop.  The local interpenetration is the visible bushing/pin
    # capture for the orthogonal gimbal axis, not a broad assembly collision.
    for suffix in ("front", "rear"):
        ctx.allow_overlap(
            outer_ring,
            cradle,
            elem_a="outer_hoop",
            elem_b=f"{suffix}_pin",
            reason="Captured trunnion pin passes through the outer hoop's bearing bore proxy.",
        )
        ctx.expect_overlap(
            outer_ring,
            cradle,
            axes="y",
            elem_a="outer_hoop",
            elem_b=f"{suffix}_pin",
            min_overlap=0.003,
            name=f"{suffix} trunnion remains inserted through outer hoop",
        )
    for suffix in ("pos", "neg"):
        ctx.allow_overlap(
            base,
            outer_ring,
            elem_a=f"side_bearing_{suffix}",
            elem_b=f"side_pin_{suffix}",
            reason="Outer gimbal axle is intentionally captured in the base side bearing bushing.",
        )

    ctx.check(
        "outer gimbal axis is left-right",
        tuple(round(v, 6) for v in outer_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={outer_joint.axis}",
    )
    ctx.check(
        "inner gimbal axis is front-back",
        tuple(round(v, 6) for v in inner_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={inner_joint.axis}",
    )
    ctx.expect_origin_gap(
        outer_ring,
        base,
        axis="z",
        min_gap=CONTROL_Z - 0.001,
        max_gap=CONTROL_Z + 0.001,
        name="outer ring center is raised above the shallow cup",
    )
    ctx.expect_origin_distance(
        outer_ring,
        cradle,
        axes="xyz",
        max_dist=0.001,
        name="two gimbal axes share the same control center",
    )
    ctx.expect_origin_distance(
        cradle,
        stick,
        axes="xyz",
        max_dist=0.001,
        name="stick is fixed at the inner cradle center",
    )

    for suffix in ("pos", "neg"):
        ctx.expect_within(
            outer_ring,
            base,
            axes="yz",
            inner_elem=f"side_pin_{suffix}",
            outer_elem=f"side_bearing_{suffix}",
            margin=0.002,
            name=f"outer side pin {suffix} is centered in base bearing",
        )
    for suffix in ("front", "rear"):
        ctx.expect_within(
            cradle,
            outer_ring,
            axes="xz",
            inner_elem=f"{suffix}_pin",
            outer_elem=f"{suffix}_bearing",
            margin=0.002,
            name=f"inner {suffix} pin is centered in outer ring bearing",
        )

    def _elem_center(part, elem_name: str) -> tuple[float, float, float] | None:
        box = ctx.part_element_world_aabb(part, elem=elem_name)
        if box is None:
            return None
        lo, hi = box
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_collar = _elem_center(stick, "collar")
    with ctx.pose({outer_joint: 0.34}):
        tipped_by_outer = _elem_center(stick, "collar")
    with ctx.pose({inner_joint: 0.34}):
        tipped_by_inner = _elem_center(stick, "collar")

    ctx.check(
        "outer ring pitch visibly carries the stick",
        rest_collar is not None
        and tipped_by_outer is not None
        and abs(tipped_by_outer[1] - rest_collar[1]) > 0.018,
        details=f"rest={rest_collar}, tipped={tipped_by_outer}",
    )
    ctx.check(
        "inner cradle roll visibly carries the stick",
        rest_collar is not None
        and tipped_by_inner is not None
        and abs(tipped_by_inner[0] - rest_collar[0]) > 0.018,
        details=f"rest={rest_collar}, tipped={tipped_by_inner}",
    )

    return ctx.report()


object_model = build_object_model()
