from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z0: float):
    """CadQuery annular cylinder in meters, with bottom at z0."""
    body = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cutter = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.020).translate(
        (0.0, 0.0, -0.010)
    )
    return body.cut(cutter).translate((0.0, 0.0, z0))


def _build_housing_shell():
    underground_sleeve = _annular_cylinder(0.145, 0.115, 0.920, -0.800)
    ground_flange = _annular_cylinder(0.300, 0.115, 0.050, -0.008)
    raised_collar = _annular_cylinder(0.180, 0.115, 0.188, 0.035)
    return underground_sleeve.union(ground_flange).union(raised_collar)


def _build_pavement_slab():
    slab = cq.Workplane("XY").rect(1.05, 1.05).extrude(0.075).translate((0.0, 0.0, -0.075))
    opening = cq.Workplane("XY").circle(0.300).extrude(0.095).translate((0.0, 0.0, -0.085))
    return slab.cut(opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_traffic_bollard")

    safety_yellow = model.material("powder_coated_safety_yellow", rgba=(1.0, 0.66, 0.04, 1.0))
    reflective_white = model.material("white_retroreflective_sheeting", rgba=(0.92, 0.98, 1.0, 1.0))
    dark_metal = model.material("blackened_steel", rgba=(0.015, 0.016, 0.017, 1.0))
    galvanized = model.material("galvanized_cast_steel", rgba=(0.45, 0.47, 0.45, 1.0))
    concrete = model.material("pavement_concrete", rgba=(0.38, 0.37, 0.34, 1.0))
    door_paint = model.material("dark_service_door_paint", rgba=(0.08, 0.09, 0.085, 1.0))

    housing = model.part("ground_housing")
    housing.visual(
        mesh_from_cadquery(_build_pavement_slab(), "pavement_slab", tolerance=0.0012),
        material=concrete,
        name="pavement_slab",
    )
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "housing_shell", tolerance=0.0008),
        material=galvanized,
        name="housing_shell",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        housing.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(0.245 * math.cos(angle), 0.245 * math.sin(angle), 0.049)),
            material=dark_metal,
            name=f"flange_bolt_{i}",
        )
    housing.visual(
        Box((0.020, 0.046, 0.110)),
        origin=Origin(xyz=(0.105, 0.0, 0.110)),
        material=dark_metal,
        name="guide_pad",
    )
    housing.visual(
        Box((0.036, 0.088, 0.155)),
        origin=Origin(xyz=(-0.105, -0.164, 0.125)),
        material=galvanized,
        name="hinge_leaf",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.095, length=1.350),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=safety_yellow,
        name="post_body",
    )
    post.visual(
        Cylinder(radius=0.103, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.772)),
        material=dark_metal,
        name="crown_cap",
    )
    for i, z in enumerate((0.505, 0.625)):
        post.visual(
            Cylinder(radius=0.1005, length=0.072),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=reflective_white,
            name=f"reflective_band_{i}",
        )

    lift = model.articulation(
        "housing_to_post",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.223)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=-0.30, upper=0.18),
    )
    lift.meta["description"] = "Main retractable post slide within the ground sleeve."

    knob = model.part("locking_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=galvanized,
        name="shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.086,
            0.045,
            body_style="lobed",
            base_diameter=0.060,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=12, depth=0.0012),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "lobed_locking_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_metal,
        name="knob_cap",
    )
    model.articulation(
        "post_to_locking_knob",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, 0.794)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.150, 0.014, 0.122)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=door_paint,
        name="door_panel",
    )
    service_door.visual(
        Cylinder(radius=0.012, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    service_door.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.120, -0.009, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="quarter_turn_latch",
    )
    model.articulation(
        "housing_to_service_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=service_door,
        origin=Origin(xyz=(-0.075, -0.195, 0.125)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("ground_housing")
    post = object_model.get_part("post")
    knob = object_model.get_part("locking_knob")
    service_door = object_model.get_part("service_door")
    lift = object_model.get_articulation("housing_to_post")
    knob_spin = object_model.get_articulation("post_to_locking_knob")
    door_hinge = object_model.get_articulation("housing_to_service_door")

    ctx.expect_within(
        post,
        housing,
        axes="xy",
        inner_elem="post_body",
        outer_elem="housing_shell",
        margin=0.0,
        name="post stays centered inside the round ground housing",
    )
    ctx.expect_overlap(
        post,
        housing,
        axes="z",
        elem_a="post_body",
        elem_b="housing_shell",
        min_overlap=0.40,
        name="post remains deeply inserted in the sleeve at the raised pose",
    )
    ctx.expect_contact(
        housing,
        post,
        elem_a="guide_pad",
        elem_b="post_body",
        contact_tol=0.001,
        name="guide pad physically supports the sliding post",
    )

    ctx.expect_gap(
        knob,
        post,
        axis="z",
        positive_elem="shaft",
        negative_elem="crown_cap",
        max_gap=0.001,
        max_penetration=0.001,
        name="locking knob shaft is seated on the post crown",
    )
    knob_type = getattr(knob_spin, "articulation_type", None)
    ctx.check(
        "locking knob uses a continuous rotary joint",
        knob_type == ArticulationType.CONTINUOUS or str(knob_type).lower().endswith("continuous"),
        details=f"type={knob_type}",
    )

    ctx.expect_contact(
        service_door,
        housing,
        elem_a="hinge_barrel",
        elem_b="hinge_leaf",
        contact_tol=0.001,
        name="service door hinge barrel is side-mounted to the collar leaf",
    )

    rest_post_pos = ctx.part_world_position(post)
    with ctx.pose({lift: 0.16}):
        raised_post_pos = ctx.part_world_position(post)
        ctx.expect_overlap(
            post,
            housing,
            axes="z",
            elem_a="post_body",
            elem_b="housing_shell",
            min_overlap=0.30,
            name="post keeps retained insertion while raised further",
        )
    ctx.check(
        "post prismatic joint translates upward",
        rest_post_pos is not None
        and raised_post_pos is not None
        and raised_post_pos[2] > rest_post_pos[2] + 0.12,
        details=f"rest={rest_post_pos}, raised={raised_post_pos}",
    )

    closed_door_aabb = ctx.part_world_aabb(service_door)
    with ctx.pose({door_hinge: 1.10}):
        open_door_aabb = ctx.part_world_aabb(service_door)
    ctx.check(
        "service door hinge opens outward from the collar",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.050,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
