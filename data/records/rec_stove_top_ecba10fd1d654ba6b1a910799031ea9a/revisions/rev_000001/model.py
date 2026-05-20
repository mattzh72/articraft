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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shallow_ceramic_cooktop")

    stone = model.material("warm_stone", rgba=(0.72, 0.70, 0.65, 1.0))
    glass = model.material("black_ceramic_glass", rgba=(0.015, 0.017, 0.020, 1.0))
    glass_edge = model.material("dark_glass_edge", rgba=(0.005, 0.006, 0.008, 1.0))
    radiant = model.material("soft_radiant_red", rgba=(1.0, 0.16, 0.03, 0.70))
    radiant_core = model.material("warm_radiant_core", rgba=(1.0, 0.48, 0.08, 0.78))
    button_mat = model.material("satin_chrome_button", rgba=(0.74, 0.74, 0.70, 1.0))
    button_face = model.material("brushed_button_face", rgba=(0.88, 0.88, 0.84, 1.0))
    dark_mark = model.material("button_mark_black", rgba=(0.02, 0.02, 0.018, 1.0))

    root = model.part("counter_cooktop")

    # A flat counter ring surrounds the black ceramic insert; there is no
    # appliance body below, only the counter slab and a shallow front control strip.
    root.visual(Box((0.950, 0.080, 0.040)), origin=Origin(xyz=(0.0, -0.288, 0.020)), material=stone, name="front_counter")
    root.visual(Box((0.950, 0.052, 0.040)), origin=Origin(xyz=(0.0, 0.334, 0.020)), material=stone, name="rear_counter")
    root.visual(Box((0.097, 0.564, 0.040)), origin=Origin(xyz=(-0.4265, 0.030, 0.020)), material=stone, name="side_counter_0")
    root.visual(Box((0.097, 0.564, 0.040)), origin=Origin(xyz=(0.4265, 0.030, 0.020)), material=stone, name="side_counter_1")

    root.visual(Box((0.764, 0.564, 0.008)), origin=Origin(xyz=(0.0, 0.030, 0.036)), material=glass, name="ceramic_glass")
    root.visual(Box((0.778, 0.014, 0.004)), origin=Origin(xyz=(0.0, -0.254, 0.038)), material=glass_edge, name="front_glass_lip")
    root.visual(Box((0.778, 0.014, 0.004)), origin=Origin(xyz=(0.0, 0.314, 0.038)), material=glass_edge, name="rear_glass_lip")
    root.visual(Box((0.014, 0.564, 0.004)), origin=Origin(xyz=(-0.386, 0.030, 0.038)), material=glass_edge, name="side_glass_lip_0")
    root.visual(Box((0.014, 0.564, 0.004)), origin=Origin(xyz=(0.386, 0.030, 0.038)), material=glass_edge, name="side_glass_lip_1")

    # Concentric glowing radiant zones laid directly on the ceramic surface.
    zone_specs = (
        (-0.185, -0.070, 0.082, "front_zone_0"),
        (0.185, -0.070, 0.072, "front_zone_1"),
        (-0.185, 0.175, 0.070, "rear_zone_0"),
        (0.185, 0.175, 0.094, "rear_zone_1"),
    )
    for x, y, radius, base_name in zone_specs:
        for idx, scale in enumerate((1.00, 0.62, 0.30)):
            root.visual(
                Cylinder(radius * scale, 0.00045),
                origin=Origin(xyz=(x, y, 0.04015 + idx * 0.00034)),
                material=radiant if idx < 2 else radiant_core,
                name=f"{base_name}_glow_{idx}",
            )
        for idx, scale in enumerate((0.82, 0.45)):
            root.visual(
                Cylinder(radius * scale, 0.00048),
                origin=Origin(xyz=(x, y, 0.04032 + idx * 0.00034)),
                material=glass,
                name=f"{base_name}_mask_{idx}",
            )

    # The front control strip is a very shallow perforated fascia.  The slot
    # liners sit behind the holes, leaving clearance for each plunger.
    strip_y = -0.3365
    root.visual(Box((0.780, 0.021, 0.012)), origin=Origin(xyz=(0.0, strip_y, 0.006)), material=glass_edge, name="strip_bottom_rail")
    root.visual(Box((0.780, 0.021, 0.010)), origin=Origin(xyz=(0.0, strip_y, 0.035)), material=glass_edge, name="strip_top_rail")

    button_centers = (-0.300, -0.150, 0.0, 0.150, 0.300)
    slot_width = 0.130
    slot_height = 0.018
    strip_min_x = -0.390
    strip_max_x = 0.390
    holes = [(center - slot_width / 2.0, center + slot_width / 2.0) for center in button_centers]
    solid_spans = [(strip_min_x, holes[0][0])]
    solid_spans.extend((holes[i][1], holes[i + 1][0]) for i in range(len(holes) - 1))
    solid_spans.append((holes[-1][1], strip_max_x))

    for idx, (x0, x1) in enumerate(solid_spans):
        root.visual(
            Box((x1 - x0, 0.021, slot_height)),
            origin=Origin(xyz=((x0 + x1) / 2.0, strip_y, 0.021)),
            material=glass_edge,
            name=f"strip_post_{idx}",
        )

    for idx, x in enumerate(button_centers):
        root.visual(
            Box((slot_width + 0.004, 0.003, slot_height + 0.004)),
            origin=Origin(xyz=(x, -0.3275, 0.021)),
            material=glass,
            name=f"slot_liner_{idx}",
        )

    # Five independent push buttons: only these parts articulate.  Their child
    # frames sit at the cap centers in the rest pose, so positive +Y is inward.
    for idx, x in enumerate(button_centers):
        button = model.part(f"button_{idx}")
        button.visual(Box((0.110, 0.014, 0.016)), origin=Origin(), material=button_mat, name="cap")
        button.visual(Box((0.098, 0.0018, 0.011)), origin=Origin(xyz=(0.0, -0.0077, 0.0)), material=button_face, name="front_face")
        # The plunger is a close guide fit in the slot and touches the upper
        # and lower slot rails, so each articulated button is physically retained.
        button.visual(Box((0.070, 0.016, 0.018)), origin=Origin(xyz=(0.0, 0.009, 0.0)), material=glass_edge, name="plunger")
        button.visual(Box((0.042, 0.0012, 0.0022)), origin=Origin(xyz=(0.0, -0.0088, 0.0043)), material=dark_mark, name="index_mark")

        model.articulation(
            f"strip_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=root,
            child=button,
            origin=Origin(xyz=(x, -0.3540, 0.021)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = list(object_model.articulations)
    ctx.check("only five button plungers articulate", len(joints) == 5, details=f"joints={[j.name for j in joints]}")

    for idx in range(5):
        button = object_model.get_part(f"button_{idx}")
        joint = object_model.get_articulation(f"strip_to_button_{idx}")
        limits = joint.motion_limits
        ctx.check(
            f"button_{idx} is inward prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == 0.006,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_within(
            button,
            "counter_cooktop",
            axes="xz",
            inner_elem="cap",
            outer_elem=f"slot_liner_{idx}",
            margin=0.010,
            name=f"button_{idx} cap aligns with its slot",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed_pos = ctx.part_world_position(button)
            ctx.expect_within(
                button,
                "counter_cooktop",
                axes="xz",
                inner_elem="plunger",
                outer_elem=f"slot_liner_{idx}",
                margin=0.004,
                name=f"button_{idx} plunger stays in slot",
            )
        ctx.check(
            f"button_{idx} moves inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.0055
            and abs(pressed_pos[0] - rest_pos[0]) < 1e-6
            and abs(pressed_pos[2] - rest_pos[2]) < 1e-6,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
