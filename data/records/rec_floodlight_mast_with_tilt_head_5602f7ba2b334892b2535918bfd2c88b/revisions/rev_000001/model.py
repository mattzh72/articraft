from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _triangular_gusset_mesh(name: str, *, side: str) -> object:
    """Closed triangular reinforcing web for the mast sleeve base."""
    thickness = 0.010
    z0 = 0.032
    z1 = 0.285
    root = 0.056
    foot = 0.178

    if side == "+x":
        pts = [(root, z0), (foot, z0), (root, z1)]
        axis = "x"
        sign = 1.0
    elif side == "-x":
        pts = [(-root, z0), (-foot, z0), (-root, z1)]
        axis = "x"
        sign = -1.0
    elif side == "+y":
        pts = [(root, z0), (foot, z0), (root, z1)]
        axis = "y"
        sign = 1.0
    elif side == "-y":
        pts = [(-root, z0), (-foot, z0), (-root, z1)]
        axis = "y"
        sign = -1.0
    else:
        raise ValueError(side)

    geom = MeshGeometry()
    if axis == "x":
        vertices = [
            (pts[0][0], -thickness / 2, pts[0][1]),
            (pts[1][0], -thickness / 2, pts[1][1]),
            (pts[2][0], -thickness / 2, pts[2][1]),
            (pts[0][0], thickness / 2, pts[0][1]),
            (pts[1][0], thickness / 2, pts[1][1]),
            (pts[2][0], thickness / 2, pts[2][1]),
        ]
    else:
        # Same wedge rotated so the plate stands on the ±Y side of the sleeve.
        vertices = [
            (-thickness / 2, pts[0][0], pts[0][1]),
            (-thickness / 2, pts[1][0], pts[1][1]),
            (-thickness / 2, pts[2][0], pts[2][1]),
            (thickness / 2, pts[0][0], pts[0][1]),
            (thickness / 2, pts[1][0], pts[1][1]),
            (thickness / 2, pts[2][0], pts[2][1]),
        ]
    for v in vertices:
        geom.add_vertex(*v)
    # two triangular faces and three rectangular side faces
    for face in (
        (0, 1, 2),
        (3, 5, 4),
        (0, 3, 4),
        (0, 4, 1),
        (1, 4, 5),
        (1, 5, 2),
        (2, 5, 3),
        (2, 3, 0),
    ):
        geom.add_face(*face)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_floodlight_mast")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    dark_steel = model.material("black_powder_coat", rgba=(0.02, 0.022, 0.024, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    bolt_metal = model.material("zinc_bolts", rgba=(0.72, 0.72, 0.68, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.62, 0.82, 1.0, 0.45))
    warm_led = model.material("warm_led_emitters", rgba=(1.0, 0.82, 0.36, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.42, 0.42, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=galvanized,
        name="base_plate",
    )
    base.visual(
        Box((0.35, 0.35, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=rubber,
        name="roof_pad",
    )

    # A hollow square lower sleeve: four real walls leave a visible clearance
    # pocket for the sliding mast rather than intersecting it as a solid block.
    sleeve_height = 0.75
    sleeve_z = 0.035 + sleeve_height / 2
    base.visual(
        Box((0.120, 0.016, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.052, sleeve_z)),
        material=galvanized,
        name="front_sleeve_wall",
    )
    base.visual(
        Box((0.120, 0.016, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.052, sleeve_z)),
        material=galvanized,
        name="rear_sleeve_wall",
    )
    base.visual(
        Box((0.016, 0.088, sleeve_height)),
        origin=Origin(xyz=(0.052, 0.0, sleeve_z)),
        material=galvanized,
        name="side_sleeve_wall_0",
    )
    base.visual(
        Box((0.016, 0.088, sleeve_height)),
        origin=Origin(xyz=(-0.052, 0.0, sleeve_z)),
        material=galvanized,
        name="side_sleeve_wall_1",
    )

    # Reinforced collar around the sleeve mouth.
    base.visual(
        Box((0.160, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, 0.070, 0.745)),
        material=galvanized,
        name="front_collar_bar",
    )
    base.visual(
        Box((0.160, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, -0.070, 0.745)),
        material=galvanized,
        name="rear_collar_bar",
    )
    base.visual(
        Box((0.020, 0.120, 0.080)),
        origin=Origin(xyz=(0.070, 0.0, 0.745)),
        material=galvanized,
        name="side_collar_bar_0",
    )
    base.visual(
        Box((0.020, 0.120, 0.080)),
        origin=Origin(xyz=(-0.070, 0.0, 0.745)),
        material=galvanized,
        name="side_collar_bar_1",
    )

    for side in ("+x", "-x", "+y", "-y"):
        base.visual(
            _triangular_gusset_mesh(f"gusset_{side.replace('+', 'p').replace('-', 'm')}", side=side),
            material=galvanized,
            name=f"gusset_{side.replace('+', 'p').replace('-', 'm')}",
        )

    for idx, (x, y) in enumerate(((-0.155, -0.155), (-0.155, 0.155), (0.155, -0.155), (0.155, 0.155))):
        base.visual(
            Cylinder(radius=0.021, length=0.009),
            origin=Origin(xyz=(x, y, 0.0395)),
            material=bolt_metal,
            name=f"anchor_bolt_{idx}",
        )

    mast = model.part("mast")
    mast.visual(
        Box((0.068, 0.068, 1.230)),
        # The part frame is at the sleeve mouth.  The tube extends downward
        # inside the lower sleeve so it remains captured at full travel.
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=galvanized,
        name="inner_tube",
    )
    for idx, z in enumerate((-0.500, -0.110)):
        mast.visual(
            Box((0.030, 0.010, 0.045)),
            origin=Origin(xyz=(0.0, 0.039, z)),
            material=rubber,
            name=f"front_slide_pad_{idx}",
        )
        mast.visual(
            Box((0.030, 0.010, 0.045)),
            origin=Origin(xyz=(0.0, -0.039, z)),
            material=rubber,
            name=f"rear_slide_pad_{idx}",
        )
        mast.visual(
            Box((0.010, 0.030, 0.045)),
            origin=Origin(xyz=(0.039, 0.0, z)),
            material=rubber,
            name=f"side_slide_pad_0_{idx}",
        )
        mast.visual(
            Box((0.010, 0.030, 0.045)),
            origin=Origin(xyz=(-0.039, 0.0, z)),
            material=rubber,
            name=f"side_slide_pad_1_{idx}",
        )
    mast.visual(
        Box((0.120, 0.120, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.6625)),
        material=galvanized,
        name="top_cap",
    )
    mast.visual(
        Box((0.240, 0.070, 0.030)),
        origin=Origin(xyz=(0.120, 0.0, 0.690)),
        material=galvanized,
        name="top_arm",
    )
    mast.visual(
        Box((0.070, 0.250, 0.035)),
        origin=Origin(xyz=(0.230, 0.0, 0.690)),
        material=galvanized,
        name="yoke_bridge",
    )
    mast.visual(
        Box((0.060, 0.018, 0.240)),
        origin=Origin(xyz=(0.230, 0.105, 0.810)),
        material=galvanized,
        name="yoke_cheek_0",
    )
    mast.visual(
        Box((0.060, 0.018, 0.240)),
        origin=Origin(xyz=(0.230, -0.105, 0.810)),
        material=galvanized,
        name="yoke_cheek_1",
    )
    for idx, y in enumerate((0.120, -0.120)):
        mast.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(0.230, y, 0.820), rpy=(math.pi / 2, 0.0, 0.0)),
            material=bolt_metal,
            name=f"hinge_bolt_cap_{idx}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.032, length=0.192),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_boss",
    )
    head.visual(
        Box((0.280, 0.180, 0.170)),
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        material=dark_steel,
        name="housing",
    )
    for idx, y in enumerate((-0.060, -0.030, 0.0, 0.030, 0.060)):
        head.visual(
            Box((0.050, 0.008, 0.140)),
            origin=Origin(xyz=(-0.005, y, 0.0)),
            material=dark_steel,
            name=f"rear_heat_sink_fin_{idx}",
        )
    head.visual(
        Box((0.012, 0.190, 0.018)),
        origin=Origin(xyz=(0.306, 0.0, 0.071)),
        material=dark_steel,
        name="front_bezel_top",
    )
    head.visual(
        Box((0.012, 0.190, 0.018)),
        origin=Origin(xyz=(0.306, 0.0, -0.071)),
        material=dark_steel,
        name="front_bezel_bottom",
    )
    head.visual(
        Box((0.012, 0.018, 0.140)),
        origin=Origin(xyz=(0.306, 0.091, 0.0)),
        material=dark_steel,
        name="front_bezel_side_0",
    )
    head.visual(
        Box((0.012, 0.018, 0.140)),
        origin=Origin(xyz=(0.306, -0.091, 0.0)),
        material=dark_steel,
        name="front_bezel_side_1",
    )
    head.visual(
        Box((0.006, 0.165, 0.125)),
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    led_positions = [(-0.045, 0.028), (0.0, 0.028), (0.045, 0.028), (-0.045, -0.028), (0.0, -0.028), (0.045, -0.028)]
    for idx, (y, z) in enumerate(led_positions):
        head.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(0.3185, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material=warm_led,
            name=f"led_chip_{idx}",
        )

    mast_slide = model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.38),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.230, 0.0, 0.820)),
        # The head projects along local +X; -Y makes positive motion tilt it upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.45, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    slide = object_model.get_articulation("mast_slide")
    tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "mast has prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and abs((slide.motion_limits.upper or 0.0) - 0.38) < 1e-6,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )
    ctx.check(
        "head has tilt hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and (tilt.motion_limits.lower or 0.0) < 0.0
        and (tilt.motion_limits.upper or 0.0) > 0.0,
        details=f"type={tilt.articulation_type}, limits={tilt.motion_limits}",
    )

    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="front_sleeve_wall",
        min_overlap=0.20,
        name="collapsed mast remains inserted in sleeve",
    )
    ctx.expect_contact(
        mast,
        base,
        elem_a="front_slide_pad_1",
        elem_b="front_sleeve_wall",
        contact_tol=0.0005,
        name="upper slide pad bears on sleeve wall",
    )
    ctx.expect_contact(
        head,
        mast,
        elem_a="trunnion_boss",
        elem_b="yoke_cheek_0",
        contact_tol=0.0005,
        name="trunnion is captured by yoke cheek",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({slide: 0.38}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="front_sleeve_wall",
            min_overlap=0.18,
            name="extended mast retains sleeve insertion",
        )
        ctx.expect_contact(
            mast,
            base,
            elem_a="front_slide_pad_0",
            elem_b="front_sleeve_wall",
            contact_tol=0.0005,
            name="extended lower slide pad still bears on sleeve",
        )
        extended_mast_pos = ctx.part_world_position(mast)

    ctx.check(
        "mast extension raises bracket",
        rest_mast_pos is not None and extended_mast_pos is not None and extended_mast_pos[2] > rest_mast_pos[2] + 0.35,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({tilt: 0.65}):
        tilted_lens_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    rest_lens_z = None if rest_lens_aabb is None else (rest_lens_aabb[0][2] + rest_lens_aabb[1][2]) / 2.0
    tilted_lens_z = None if tilted_lens_aabb is None else (tilted_lens_aabb[0][2] + tilted_lens_aabb[1][2]) / 2.0
    ctx.check(
        "positive head tilt raises floodlight lens",
        rest_lens_z is not None and tilted_lens_z is not None and tilted_lens_z > rest_lens_z + 0.12,
        details=f"rest_lens_z={rest_lens_z}, tilted_lens_z={tilted_lens_z}",
    )

    return ctx.report()


object_model = build_object_model()
