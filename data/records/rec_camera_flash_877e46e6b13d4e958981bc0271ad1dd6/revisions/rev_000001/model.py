from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _chamfered_box(size: tuple[float, float, float], chamfer: float):
    """CadQuery rounded/chamfered rectangular shell used for molded flash plastics."""
    return cq.Workplane("XY").box(*size).edges().chamfer(chamfer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_flash")

    matte_black = Material("matte_black", rgba=(0.010, 0.010, 0.012, 1.0))
    satin_black = Material("satin_black", rgba=(0.030, 0.032, 0.035, 1.0))
    dark_panel = Material("dark_panel", rgba=(0.018, 0.020, 0.024, 1.0))
    rubber = Material("rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    lcd_glass = Material("lcd_glass", rgba=(0.030, 0.070, 0.095, 0.88))
    flash_lens = Material("flash_lens", rgba=(0.86, 0.90, 0.86, 0.62))
    metal = Material("brushed_metal", rgba=(0.56, 0.56, 0.52, 1.0))
    white_mark = Material("white_mark", rgba=(0.92, 0.92, 0.84, 1.0))

    body = model.part("body")

    # Broad DSLR hot-shoe mounting foot and locking collar.
    body.visual(
        Box((0.060, 0.084, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_black,
        name="foot_base",
    )
    body.visual(
        Box((0.050, 0.064, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=metal,
        name="shoe_plate",
    )
    for rail_y in (-0.036, 0.036):
        body.visual(
            Box((0.052, 0.006, 0.005)),
            origin=Origin(xyz=(0.0, rail_y, 0.0145)),
            material=metal,
            name=f"shoe_rail_{0 if rail_y < 0 else 1}",
        )
    body.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=rubber,
        name="lock_collar",
    )

    # Tall control body with molded chamfers.
    body.visual(
        mesh_from_cadquery(_chamfered_box((0.056, 0.070, 0.135), 0.004), "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=matte_black,
        name="body_shell",
    )

    # Rear interface panel: LCD bezel at top, selector controls below.
    body.visual(
        Box((0.0024, 0.061, 0.119)),
        origin=Origin(xyz=(-0.0292, 0.0, 0.087)),
        material=dark_panel,
        name="rear_panel",
    )
    body.visual(
        Box((0.0012, 0.044, 0.024)),
        origin=Origin(xyz=(-0.0310, 0.0, 0.116)),
        material=lcd_glass,
        name="lcd_window",
    )
    body.visual(
        Box((0.0018, 0.054, 0.004)),
        origin=Origin(xyz=(-0.0313, 0.0, 0.131)),
        material=rubber,
        name="lcd_top_bezel",
    )
    body.visual(
        Box((0.0018, 0.054, 0.004)),
        origin=Origin(xyz=(-0.0313, 0.0, 0.101)),
        material=rubber,
        name="lcd_bottom_bezel",
    )
    for bezel_y in (-0.027, 0.027):
        body.visual(
            Box((0.0018, 0.004, 0.034)),
            origin=Origin(xyz=(-0.0313, bezel_y, 0.116)),
            material=rubber,
            name=f"lcd_side_bezel_{0 if bezel_y < 0 else 1}",
        )

    # A shallow control recess behind the articulated dial and keys.
    body.visual(
        Cylinder(radius=0.021, length=0.0016),
        origin=Origin(xyz=(-0.0311, 0.0, 0.063), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=satin_black,
        name="dial_recess",
    )

    # Swivelling upper yoke: turntable plus a trunnion yoke cradling the head.
    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_black,
        name="swivel_turntable",
    )
    yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.130, 0.052, 0.070),
                span_width=0.110,
                trunnion_diameter=0.016,
                trunnion_center_z=0.044,
                base_thickness=0.014,
                corner_radius=0.003,
                center=False,
            ),
            "head_yoke",
        ),
        # The helper spans its cheeks along local X; yawing it puts the short
        # yoke arms on the camera-flash sides (world +/-Y).
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=matte_black,
        name="head_yoke",
    )

    # Forward-facing flash head, pitched about its side trunnion barrel.
    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_chamfered_box((0.078, 0.096, 0.052), 0.005), "head_shell"),
        origin=Origin(xyz=(0.043, 0.0, 0.000)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.003, 0.082, 0.037)),
        origin=Origin(xyz=(0.0835, 0.0, 0.000)),
        material=flash_lens,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.110),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="pitch_trunnion",
    )
    head.visual(
        Box((0.012, 0.090, 0.005)),
        origin=Origin(xyz=(0.0855, 0.0, 0.0235)),
        material=satin_black,
        name="front_lip",
    )

    # Rear selector dial with a tactile ribbed rim and marker.
    dial = model.part("selector_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.008,
                body_style="faceted",
                base_diameter=0.032,
                top_diameter=0.026,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=20, depth=0.0006, width=0.0012),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=90.0),
            ),
            "selector_dial_cap",
        ),
        origin=Origin(xyz=(-0.0040, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=satin_black,
        name="dial_cap",
    )
    dial.visual(
        Box((0.0010, 0.0040, 0.0100)),
        origin=Origin(xyz=(-0.0084, 0.0, 0.0105)),
        material=white_mark,
        name="dial_marker",
    )

    key_specs = {
        "top_key": ((0.006, 0.020, 0.010), (0.0, 0.087)),
        "bottom_key": ((0.006, 0.020, 0.010), (0.0, 0.039)),
        "side_key_0": ((0.006, 0.010, 0.018), (-0.027, 0.063)),
        "side_key_1": ((0.006, 0.010, 0.018), (0.027, 0.063)),
    }
    keys = {}
    for key_name, (key_size, (key_y, key_z)) in key_specs.items():
        key = model.part(key_name)
        key.visual(
            Box(key_size),
            origin=Origin(xyz=(-key_size[0] / 2.0, 0.0, 0.0)),
            material=rubber,
            name="key_cap",
        )
        keys[key_name] = (key, key_y, key_z)

    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.1535)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=-0.45, upper=1.55),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(-0.0304, 0.0, 0.063)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )
    for key_name, (key, key_y, key_z) in keys.items():
        model.articulation(
            f"body_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(-0.0304, key_y, key_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    dial = object_model.get_part("selector_dial")
    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.check(
        "head assembly has vertical swivel",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits.lower < -2.0
        and swivel.motion_limits.upper > 2.0,
    )
    ctx.check(
        "head pitches on side yoke axis",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(pitch.axis) == (0.0, -1.0, 0.0)
        and pitch.motion_limits.upper > 1.2,
    )
    ctx.check(
        "selector dial is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(dial_joint.axis) == (-1.0, 0.0, 0.0),
    )

    ctx.expect_within(
        head,
        yoke,
        axes="y",
        inner_elem="pitch_trunnion",
        outer_elem="head_yoke",
        margin=0.001,
        name="pitch trunnion sits between yoke cheeks",
    )
    ctx.allow_overlap(
        head,
        yoke,
        elem_a="pitch_trunnion",
        elem_b="head_yoke",
        reason="The head's trunnion barrel is intentionally captured inside the yoke bore proxy.",
    )
    ctx.expect_overlap(
        head,
        yoke,
        axes="z",
        elem_a="pitch_trunnion",
        elem_b="head_yoke",
        min_overlap=0.010,
        name="trunnion aligns with yoke bore height",
    )
    ctx.expect_gap(
        yoke,
        body,
        axis="z",
        positive_elem="swivel_turntable",
        negative_elem="body_shell",
        max_gap=0.002,
        max_penetration=0.0,
        name="swivel turntable seats on body top",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="x",
        positive_elem="rear_panel",
        negative_elem="dial_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="selector dial sits proud on rear panel",
    )

    for key_name in ("top_key", "bottom_key", "side_key_0", "side_key_1"):
        key = object_model.get_part(key_name)
        joint = object_model.get_articulation(f"body_to_{key_name}")
        ctx.check(
            f"{key_name} is a push button",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (1.0, 0.0, 0.0)
            and joint.motion_limits.upper >= 0.0035,
        )
        ctx.expect_gap(
            body,
            key,
            axis="x",
            positive_elem="rear_panel",
            negative_elem="key_cap",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{key_name} rests on rear interface",
        )

    rest_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({pitch: pitch.motion_limits.upper}):
        raised_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "positive pitch tilts flash head upward",
        rest_lens is not None
        and raised_lens is not None
        and raised_lens[1][2] > rest_lens[1][2] + 0.035,
        details=f"rest={rest_lens}, pitched={raised_lens}",
    )

    rest_marker = ctx.part_element_world_aabb(dial, elem="dial_marker")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        turned_marker = ctx.part_element_world_aabb(dial, elem="dial_marker")
    ctx.check(
        "dial marker rotates around rear center",
        rest_marker is not None
        and turned_marker is not None
        and abs(((turned_marker[0][1] + turned_marker[1][1]) / 2.0) - ((rest_marker[0][1] + rest_marker[1][1]) / 2.0)) > 0.006,
        details=f"rest={rest_marker}, turned={turned_marker}",
    )

    key = object_model.get_part("top_key")
    key_joint = object_model.get_articulation("body_to_top_key")
    rest_key_pos = ctx.part_world_position(key)
    with ctx.pose({key_joint: key_joint.motion_limits.upper}):
        pushed_key_pos = ctx.part_world_position(key)
    ctx.check(
        "top key travels inward",
        rest_key_pos is not None
        and pushed_key_pos is not None
        and pushed_key_pos[0] > rest_key_pos[0] + 0.003,
        details=f"rest={rest_key_pos}, pushed={pushed_key_pos}",
    )

    return ctx.report()


object_model = build_object_model()
