from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SLEEVE_LENGTH = 1.25
SLEEVE_INNER_RADIUS = 0.105
SLEEVE_OUTER_RADIUS = 0.130
PLATE_RADIUS = 0.180
PLATE_THICKNESS = 0.025
POST_OUTER_RADIUS = 0.085
POST_INNER_RADIUS = 0.060
POST_LENGTH = 1.15
POST_BOTTOM_Z = -0.35
POST_TOP_Z = POST_BOTTOM_Z + POST_LENGTH
CAP_HEIGHT = 0.055
CAP_RADIUS = 0.095
CAP_BOSS_RADIUS = 0.045
CAP_BOSS_LENGTH = 0.070
POST_RETRACTED_Q = -(POST_TOP_Z + CAP_HEIGHT - 0.005)
POST_EXTENDED_Q = 0.05


def _annular_extrusion(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    height: float,
) -> cq.Workplane:
    """Return a vertical tube/ring whose lower face starts at z_min."""
    return (
        cq.Workplane("XY")
        .workplane(offset=z_min)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _pavement_patch() -> cq.Workplane:
    """Square concrete patch with a circular pocket for the flush metal plate."""
    slab = cq.Workplane("XY").box(0.78, 0.78, 0.060).translate((0.0, 0.0, -0.030))
    pocket_cutter = (
        cq.Workplane("XY")
        .workplane(offset=-0.070)
        .circle(PLATE_RADIUS - 0.003)
        .extrude(0.090)
    )
    return slab.cut(pocket_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_bollard")

    concrete = model.material("cast_concrete", color=(0.47, 0.48, 0.46, 1.0))
    dark_steel = model.material("dark_sleeve_steel", color=(0.03, 0.035, 0.04, 1.0))
    plate_steel = model.material("brushed_plate_steel", color=(0.55, 0.57, 0.56, 1.0))
    post_steel = model.material("brushed_stainless_post", color=(0.72, 0.74, 0.72, 1.0))
    black_rubber = model.material("black_rubber", color=(0.005, 0.005, 0.004, 1.0))
    reflector = model.material("yellow_reflective_band", color=(1.0, 0.74, 0.06, 1.0))
    cap_steel = model.material("darker_keyed_cap", color=(0.30, 0.32, 0.33, 1.0))
    key_black = model.material("black_keyway", color=(0.0, 0.0, 0.0, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_pavement_patch(), "pavement_patch"),
        material=concrete,
        name="pavement_patch",
    )
    sleeve.visual(
        mesh_from_cadquery(
            _annular_extrusion(
                SLEEVE_OUTER_RADIUS,
                SLEEVE_INNER_RADIUS,
                -SLEEVE_LENGTH,
                SLEEVE_LENGTH,
            ),
            "sleeve_wall",
        ),
        material=dark_steel,
        name="sleeve_wall",
    )
    sleeve.visual(
        mesh_from_cadquery(
            _annular_extrusion(
                PLATE_RADIUS,
                SLEEVE_INNER_RADIUS,
                -PLATE_THICKNESS,
                PLATE_THICKNESS,
            ),
            "flush_plate",
        ),
        material=plate_steel,
        name="flush_plate",
    )
    sleeve.visual(
        mesh_from_cadquery(
            _annular_extrusion(0.116, CAP_RADIUS + 0.003, -0.010, 0.012),
            "mouth_gasket",
        ),
        material=black_rubber,
        name="mouth_gasket",
    )
    guide_depth = SLEEVE_INNER_RADIUS - POST_OUTER_RADIUS + 0.004
    guide_center = POST_OUTER_RADIUS + guide_depth / 2.0
    for name, xyz, size in (
        ("guide_pad_0", (guide_center, 0.0, -0.085), (guide_depth, 0.028, 0.050)),
        ("guide_pad_1", (-guide_center, 0.0, -0.085), (guide_depth, 0.028, 0.050)),
        ("guide_pad_2", (0.0, guide_center, -0.085), (0.028, guide_depth, 0.050)),
        ("guide_pad_3", (0.0, -guide_center, -0.085), (0.028, guide_depth, 0.050)),
    ):
        sleeve.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=black_rubber,
            name=name,
        )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(
            _annular_extrusion(
                POST_OUTER_RADIUS,
                POST_INNER_RADIUS,
                POST_BOTTOM_Z,
                POST_LENGTH,
            ),
            "post_tube",
        ),
        material=post_steel,
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=POST_OUTER_RADIUS + 0.003, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=reflector,
        name="lower_reflector",
    )
    post.visual(
        Cylinder(radius=POST_OUTER_RADIUS + 0.003, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=reflector,
        name="upper_reflector",
    )
    post.visual(
        Cylinder(radius=POST_OUTER_RADIUS + 0.002, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black_rubber,
        name="base_wiper",
    )

    keyed_cap = model.part("keyed_cap")
    keyed_cap.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
        material=cap_steel,
        name="cap_disk",
    )
    keyed_cap.visual(
        Cylinder(radius=CAP_BOSS_RADIUS, length=CAP_BOSS_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -CAP_BOSS_LENGTH / 2.0 + 0.001)),
        material=cap_steel,
        name="cap_boss",
    )
    keyed_cap.visual(
        Cylinder(radius=CAP_RADIUS + 0.0015, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=black_rubber,
        name="separation_seam",
    )
    keyed_cap.visual(
        Box((0.082, 0.018, 0.004)),
        origin=Origin(xyz=(0.012, 0.0, CAP_HEIGHT + 0.001)),
        material=key_black,
        name="key_slot",
    )
    keyed_cap.visual(
        Box((0.018, 0.052, 0.004)),
        origin=Origin(xyz=(-0.026, 0.0, CAP_HEIGHT + 0.0015)),
        material=key_black,
        name="key_stem",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=POST_RETRACTED_Q,
            upper=POST_EXTENDED_Q,
            effort=900.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "post_to_keyed_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=keyed_cap,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    keyed_cap = object_model.get_part("keyed_cap")
    slide = object_model.get_articulation("sleeve_to_post")
    cap_spin = object_model.get_articulation("post_to_keyed_cap")

    ctx.check(
        "post has a prismatic sleeve slide",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "keyed cap has continuous rotation",
        cap_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={cap_spin.articulation_type}",
    )
    ctx.check(
        "cap rotation axis is vertical",
        tuple(round(v, 6) for v in cap_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={cap_spin.axis}",
    )

    ctx.expect_gap(
        keyed_cap,
        post,
        axis="z",
        positive_elem="cap_disk",
        negative_elem="post_tube",
        max_gap=0.001,
        max_penetration=1e-6,
        name="keyed cap sits on post crown without merging",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_tube",
        elem_b="sleeve_wall",
        min_overlap=0.30,
        name="extended post remains retained in sleeve",
    )

    rest_cap_aabb = ctx.part_element_world_aabb(keyed_cap, elem="cap_disk")
    with ctx.pose({slide: POST_RETRACTED_Q}):
        retracted_cap_aabb = ctx.part_element_world_aabb(keyed_cap, elem="cap_disk")
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_tube",
            elem_b="sleeve_wall",
            min_overlap=1.05,
            name="retracted post is still captured by underground sleeve",
        )
    with ctx.pose({cap_spin: math.pi * 1.5}):
        ctx.expect_gap(
            keyed_cap,
            post,
            axis="z",
            positive_elem="cap_disk",
            negative_elem="post_tube",
            max_gap=0.001,
            max_penetration=1e-6,
            name="rotated keyed cap stays seated on crown",
        )

    rest_top = rest_cap_aabb[1][2] if rest_cap_aabb is not None else None
    retracted_top = retracted_cap_aabb[1][2] if retracted_cap_aabb is not None else None
    ctx.check(
        "post retracts to a flush keyed top cap",
        rest_top is not None
        and retracted_top is not None
        and rest_top > 0.80
        and abs(retracted_top - 0.005) <= 0.012,
        details=f"extended cap top={rest_top}, retracted cap top={retracted_top}",
    )

    return ctx.report()


object_model = build_object_model()
