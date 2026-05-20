from __future__ import annotations

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
import cadquery as cq


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """CadQuery tube/ring extruded upward from the local XY plane."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _sleeve_shell() -> cq.Workplane:
    """One connected metal sleeve: underground tube, flange collar, and raised lip."""
    underground_tube = _annular_cylinder(0.145, 0.126, 1.05).translate((0.0, 0.0, -0.97))
    flange = _annular_cylinder(0.240, 0.128, 0.045).translate((0.0, 0.0, 0.0))
    raised_lip = _annular_cylinder(0.158, 0.126, 0.040).translate((0.0, 0.0, 0.045))
    return underground_tube.union(flange).union(raised_lip)


def _ground_pad() -> cq.Workplane:
    """A small cut-away patch of street surface with a hole around the sleeve."""
    pad = cq.Workplane("XY").box(0.85, 0.85, 0.080).translate((0.0, 0.0, -0.040))
    hole = cq.Workplane("XY").circle(0.140).extrude(0.120).translate((0.0, 0.0, -0.100))
    return pad.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_security_bollard")

    dark_steel = model.material("dark_galvanized_steel", rgba=(0.06, 0.065, 0.065, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.46, 0.48, 0.47, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.43, 0.42, 0.39, 1.0))
    reflective = model.material("yellow_reflector", rgba=(1.0, 0.78, 0.12, 1.0))
    black = model.material("black_key_slot", rgba=(0.005, 0.005, 0.004, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_ground_pad(), "ground_pad", tolerance=0.002, angular_tolerance=0.08),
        material=concrete,
        name="ground_pad",
    )
    sleeve.visual(
        mesh_from_cadquery(_sleeve_shell(), "sleeve_shell", tolerance=0.001, angular_tolerance=0.05),
        material=brushed_steel,
        name="sleeve_shell",
    )
    # Four polymer guide shoes make the post read as mechanically captured by the
    # collar while still leaving the tube hollow for the sliding travel.
    for x, y, size, name in (
        (0.1165, 0.0, (0.023, 0.052, 0.080), "guide_shoe_0"),
        (-0.1165, 0.0, (0.023, 0.052, 0.080), "guide_shoe_1"),
        (0.0, 0.1165, (0.052, 0.023, 0.080), "guide_shoe_2"),
        (0.0, -0.1165, (0.052, 0.023, 0.080), "guide_shoe_3"),
    ):
        sleeve.visual(
            Box(size),
            origin=Origin(xyz=(x, y, 0.040)),
            material=black,
            name=name,
        )

    post = model.part("post")
    # The post is split into flush cylindrical sections so the yellow bands read
    # as painted reflective sleeves rather than thicker collars that would jam in
    # the guide shoes when the bollard retracts.
    for center_z, length, material, name in (
        (0.07875, 0.81750, dark_steel, "post_body"),
        (0.51000, 0.04500, reflective, "reflector_band_0"),
        (0.58500, 0.10500, dark_steel, "post_mid"),
        (0.66000, 0.04500, reflective, "reflector_band_1"),
        (0.75125, 0.13750, dark_steel, "post_top"),
    ):
        post.visual(
            Cylinder(radius=0.105, length=length),
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            material=material,
            name=name,
        )

    cap = model.part("key_cap")
    cap.visual(
        Cylinder(radius=0.052, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=brushed_steel,
        name="cap_disk",
    )
    cap.visual(
        Box((0.070, 0.013, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=black,
        name="key_slot",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.25, lower=-0.720, upper=0.180),
    )

    model.articulation(
        "post_to_key_cap",
        ArticulationType.REVOLUTE,
        parent=post,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.5708),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    cap = object_model.get_part("key_cap")
    slide = object_model.get_articulation("sleeve_to_post")
    cap_turn = object_model.get_articulation("post_to_key_cap")

    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="post_body",
        outer_elem="sleeve_shell",
        margin=0.0,
        name="post centered inside round sleeve opening",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_body",
        elem_b="sleeve_shell",
        min_overlap=0.140,
        name="raised post remains inserted in sleeve",
    )
    ctx.expect_contact(
        cap,
        post,
        elem_a="cap_disk",
        elem_b="post_top",
        contact_tol=0.001,
        name="key cap sits on top of post",
    )

    rest_pos = ctx.part_world_position(post)
    with ctx.pose({slide: 0.180}):
        raised_pos = ctx.part_world_position(post)
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_body",
            elem_b="sleeve_shell",
            min_overlap=0.120,
            name="fully raised post retains sleeve engagement",
        )
    with ctx.pose({slide: -0.720}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_body",
            elem_b="sleeve_shell",
            min_overlap=0.800,
            name="retracted post stores inside underground sleeve",
        )
    ctx.check(
        "post slides upward on positive travel",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.15,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_slot = ctx.part_element_world_aabb(cap, elem="key_slot")
    with ctx.pose({cap_turn: 1.5708}):
        turned_slot = ctx.part_element_world_aabb(cap, elem="key_slot")
    if rest_slot is not None and turned_slot is not None:
        rest_dx = rest_slot[1][0] - rest_slot[0][0]
        rest_dy = rest_slot[1][1] - rest_slot[0][1]
        turn_dx = turned_slot[1][0] - turned_slot[0][0]
        turn_dy = turned_slot[1][1] - turned_slot[0][1]
        slot_rotated = rest_dx > rest_dy * 3.0 and turn_dy > turn_dx * 3.0
        details = f"rest_dx={rest_dx:.3f}, rest_dy={rest_dy:.3f}, turn_dx={turn_dx:.3f}, turn_dy={turn_dy:.3f}"
    else:
        slot_rotated = False
        details = f"rest_slot={rest_slot}, turned_slot={turned_slot}"
    ctx.check("key slot rotates a quarter turn", slot_rotated, details=details)

    return ctx.report()


object_model = build_object_model()
