from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BARREL_Y = 0.046
EYE_CUP_TRAVEL = 0.012


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> MeshGeometry:
    geom = CylinderGeometry(radius, length, radial_segments=40)
    geom.rotate_y(math.pi / 2.0)
    geom.translate(*center)
    return geom


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> MeshGeometry:
    geom = BoxGeometry(size)
    geom.translate(*center)
    return geom


def _barrel_body(sign: float) -> MeshGeometry:
    """Rubber-armored straight-through roof-prism barrel with hinge lugs."""
    y = sign * BARREL_Y
    outer_y = sign * 0.069
    geom = MeshGeometry()

    # Main straight-through tube, front objective bell, and roof-prism shoulder.
    geom.merge(_cylinder_x(0.022, 0.145, (0.012, y, 0.0)))
    geom.merge(_cylinder_x(0.028, 0.032, (0.083, y, 0.0)))
    geom.merge(_box((0.086, 0.036, 0.030), (-0.004, y, 0.021)))
    geom.merge(_box((0.060, 0.026, 0.010), (-0.015, y, 0.040)))

    # Two compact hinge leaves reach in to the central hinge pin.
    for x in (-0.037, 0.040):
        geom.merge(_box((0.034, 0.028, 0.009), (x, sign * 0.020, 0.0)))

    # Raised rubber grip ribs on the outboard side.
    for x in (-0.030, -0.014, 0.002, 0.018, 0.034):
        geom.merge(_box((0.004, 0.008, 0.025), (x, outer_y, 0.006)))

    return geom


def _eyecup_shell() -> MeshGeometry:
    """Hollow sliding rubber eyecup, axis along local -X from its joint frame."""
    length = 0.026
    outer = [
        (0.0200, 0.000),
        (0.0210, 0.005),
        (0.0210, 0.018),
        (0.0240, length),
    ]
    inner = [
        # The soft rubber bore slightly compresses on the eyepiece collar so the
        # separate prismatic cup reads as physically retained rather than loose.
        (0.0160, 0.000),
        (0.0160, length),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    # Rotate the lathe's +Z length so the eyecup extends rearward along -X.
    geom.rotate_y(-math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_prism_birdwatching_binocular")

    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.016, 0.014, 1.0))
    dark = model.material("dark_graphite", rgba=(0.06, 0.065, 0.060, 1.0))
    glass = model.material("green_coated_glass", rgba=(0.10, 0.32, 0.28, 0.55))
    hinge_metal = model.material("satin_black_hinge", rgba=(0.025, 0.027, 0.025, 1.0))

    central_hinge = model.part("central_hinge")
    central_hinge.visual(
        Cylinder(radius=0.006, length=0.165),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="hinge_pin",
    )
    central_hinge.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="rear_hinge_cap",
    )
    central_hinge.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="front_hinge_cap",
    )
    central_hinge.visual(
        Box((0.026, 0.014, 0.030)),
        origin=Origin(xyz=(-0.030, 0.0, 0.014)),
        material=hinge_metal,
        name="focus_pedestal",
    )
    central_hinge.visual(
        Box((0.026, 0.048, 0.008)),
        origin=Origin(xyz=(-0.030, 0.0, 0.023)),
        material=hinge_metal,
        name="focus_crossbar",
    )
    for index, y in enumerate((-0.024, 0.024)):
        central_hinge.visual(
            Box((0.026, 0.006, 0.040)),
            origin=Origin(xyz=(-0.030, y, 0.046)),
            material=hinge_metal,
            name=f"focus_yoke_{index}",
        )

    for idx, sign in enumerate((1.0, -1.0)):
        barrel = model.part(f"barrel_{idx}")
        y = sign * BARREL_Y
        barrel.visual(
            mesh_from_geometry(_barrel_body(sign), f"barrel_{idx}_body"),
            material=rubber,
            name="body_shell",
        )
        barrel.visual(
            Cylinder(radius=0.0165, length=0.060),
            origin=Origin(xyz=(-0.082, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="eyepiece_collar",
        )
        barrel.visual(
            Cylinder(radius=0.023, length=0.004),
            origin=Origin(xyz=(0.097, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name="objective_glass",
        )
        barrel.visual(
            Cylinder(radius=0.012, length=0.003),
            origin=Origin(xyz=(-0.112, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name="ocular_glass",
        )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.037,
                body_style="cylindrical",
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=28, depth=0.0011),
            ),
            "center_focus_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="ribbed_wheel",
    )
    focus_knob.visual(
        Cylinder(radius=0.0055, length=0.049),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="focus_axle",
    )

    for idx in (0, 1):
        eyecup = model.part(f"eyecup_{idx}")
        eyecup.visual(
            mesh_from_geometry(_eyecup_shell(), f"eyecup_{idx}_rubber"),
            material=rubber,
            name="rubber_cup",
        )

    hinge_0 = model.articulation(
        "hinge_to_barrel_0",
        ArticulationType.REVOLUTE,
        parent=central_hinge,
        child="barrel_0",
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.8, lower=-0.12, upper=0.12),
    )
    model.articulation(
        "hinge_to_barrel_1",
        ArticulationType.REVOLUTE,
        parent=central_hinge,
        child="barrel_1",
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.8, lower=-0.12, upper=0.12),
        mimic=Mimic(joint=hinge_0.name, multiplier=-1.0, offset=0.0),
    )

    model.articulation(
        "barrel_0_to_eyecup",
        ArticulationType.PRISMATIC,
        parent="barrel_0",
        child="eyecup_0",
        origin=Origin(xyz=(-0.092, BARREL_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=EYE_CUP_TRAVEL),
    )
    model.articulation(
        "barrel_1_to_eyecup",
        ArticulationType.PRISMATIC,
        parent="barrel_1",
        child="eyecup_1",
        origin=Origin(xyz=(-0.092, -BARREL_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=EYE_CUP_TRAVEL),
    )
    model.articulation(
        "hinge_to_focus_knob",
        ArticulationType.REVOLUTE,
        parent=central_hinge,
        child=focus_knob,
        origin=Origin(xyz=(-0.030, 0.0, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel_0 = object_model.get_part("barrel_0")
    barrel_1 = object_model.get_part("barrel_1")
    eyecup_0 = object_model.get_part("eyecup_0")
    eyecup_1 = object_model.get_part("eyecup_1")
    slide_0 = object_model.get_articulation("barrel_0_to_eyecup")
    slide_1 = object_model.get_articulation("barrel_1_to_eyecup")
    hinge_0 = object_model.get_articulation("hinge_to_barrel_0")

    for idx in (0, 1):
        ctx.allow_overlap(
            f"barrel_{idx}",
            f"eyecup_{idx}",
            elem_a="eyepiece_collar",
            elem_b="rubber_cup",
            reason="The sliding rubber eyecup bore is intentionally modeled with slight compression on the eyepiece collar.",
        )
    for yoke in ("focus_yoke_0", "focus_yoke_1"):
        ctx.allow_overlap(
            "central_hinge",
            "focus_knob",
            elem_a=yoke,
            elem_b="focus_axle",
            reason="The focus wheel axle is intentionally captured in the yoke cheek bearing.",
        )
        ctx.expect_overlap(
            "central_hinge",
            "focus_knob",
            axes="y",
            min_overlap=0.001,
            elem_a=yoke,
            elem_b="focus_axle",
            name=f"{yoke} captures focus axle",
        )

    # The two roof-prism barrels should be parallel, separated, and retained
    # about the central longitudinal hinge.
    ctx.expect_overlap(
        barrel_0,
        barrel_1,
        axes="x",
        min_overlap=0.13,
        elem_a="body_shell",
        elem_b="body_shell",
        name="straight-through barrels overlap along the optical axis",
    )
    ctx.expect_origin_distance(
        barrel_0,
        barrel_1,
        axes="xy",
        max_dist=0.001,
        name="barrel frames share the central hinge axis",
    )

    for idx, (barrel, eyecup, slide) in enumerate(
        ((barrel_0, eyecup_0, slide_0), (barrel_1, eyecup_1, slide_1))
    ):
        ctx.expect_overlap(
            eyecup,
            barrel,
            axes="x",
            min_overlap=0.018,
            elem_a="rubber_cup",
            elem_b="eyepiece_collar",
            name=f"eyecup_{idx} remains sleeved on eyepiece at rest",
        )
        rest_pos = ctx.part_world_position(eyecup)
        with ctx.pose({slide: EYE_CUP_TRAVEL}):
            ctx.expect_overlap(
                eyecup,
                barrel,
                axes="x",
                min_overlap=0.006,
                elem_a="rubber_cup",
                elem_b="eyepiece_collar",
                name=f"eyecup_{idx} keeps insertion when extended",
            )
            extended_pos = ctx.part_world_position(eyecup)
        ctx.check(
            f"eyecup_{idx} slides rearward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] < rest_pos[0] - 0.010,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    rest_aabb = ctx.part_world_aabb(barrel_0)
    with ctx.pose({hinge_0: 0.10}):
        splayed_aabb = ctx.part_world_aabb(barrel_0)
    ctx.check(
        "central hinge changes barrel attitude",
        rest_aabb is not None
        and splayed_aabb is not None
        and splayed_aabb[1][2] > rest_aabb[1][2] + 0.004,
        details=f"rest={rest_aabb}, splayed={splayed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
