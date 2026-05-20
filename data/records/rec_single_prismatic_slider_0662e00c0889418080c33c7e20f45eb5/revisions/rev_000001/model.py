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
    model = ArticulatedObject(name="rail_slider")

    # Materials
    model.material("rail_steel", rgba=(0.40, 0.42, 0.45, 1.0))
    model.material("carriage_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("handle_blue", rgba=(0.20, 0.40, 0.80, 1.0))

    # Fixed Rail (root part)
    rail = model.part("rail")
    
    # Main rail body: 0.6m long, 0.1m wide, 0.02m high
    # Rail part origin at bottom center (z=0 is bottom of rail)
    rail.visual(
        Box((0.60, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material="rail_steel",
        name="rail_body",
    )
    
    # Left side guide lip: extends upward to capture carriage
    # Lip height 0.04m, so top at z=0.06 (above carriage height)
    # Carriage bottom at z=0.02, top at z=0.05, so lips must extend above 0.05
    rail.visual(
        Box((0.60, 0.008, 0.04)),
        origin=Origin(xyz=(0.0, 0.046, 0.04)),
        material="rail_steel",
        name="left_lip",
    )
    
    # Right side guide lip
    rail.visual(
        Box((0.60, 0.008, 0.04)),
        origin=Origin(xyz=(0.0, -0.046, 0.04)),
        material="rail_steel",
        name="right_lip",
    )
    
    # Left end stop
    rail.visual(
        Box((0.015, 0.10, 0.04)),
        origin=Origin(xyz=(-0.2925, 0.0, 0.02)),
        material="rail_steel",
        name="left_end_stop",
    )
    
    # Right end stop
    rail.visual(
        Box((0.015, 0.10, 0.04)),
        origin=Origin(xyz=(0.2925, 0.0, 0.02)),
        material="rail_steel",
        name="right_end_stop",
    )

    # Sliding Carriage
    carriage = model.part("carriage")
    
    # Main carriage block: 0.12m long, 0.07m wide, 0.03m high
    # Carriage part origin at center of carriage block
    # Bottom sits on rail at z=0.02, so center at z=0.035 in rail frame
    carriage.visual(
        Box((0.12, 0.07, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="carriage_steel",
        name="carriage_block",
    )
    
    # Handle tab on top of carriage
    carriage.visual(
        Box((0.03, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="handle_blue",
        name="handle_tab",
    )

    # Prismatic joint: carriage slides along X-axis on rail
    # Joint origin at carriage part origin (center of carriage block)
    # At q=0, carriage is centered on rail (x=0)
    # Carriage bottom at z=0.02 (sits on rail), carriage center at z=0.035
    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.5,
            lower=-0.22,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    slider = object_model.get_articulation("rail_to_carriage")

    # Verify rail has all required visual elements
    ctx.check(
        "rail has body",
        rail.get_visual("rail_body") is not None,
        details="Rail body visual missing",
    )
    ctx.check(
        "rail has left guide lip",
        rail.get_visual("left_lip") is not None,
        details="Left guide lip missing",
    )
    ctx.check(
        "rail has right guide lip",
        rail.get_visual("right_lip") is not None,
        details="Right guide lip missing",
    )
    ctx.check(
        "rail has end stops",
        rail.get_visual("left_end_stop") is not None
        and rail.get_visual("right_end_stop") is not None,
        details="End stops missing",
    )

    # Verify carriage has required visual elements
    ctx.check(
        "carriage has block",
        carriage.get_visual("carriage_block") is not None,
        details="Carriage block missing",
    )
    ctx.check(
        "carriage has handle tab",
        carriage.get_visual("handle_tab") is not None,
        details="Handle tab missing",
    )

    # Check carriage is supported on rail at rest position (q=0)
    with ctx.pose({slider: 0.0}):
        # Carriage should be centered on rail along X at rest
        ctx.expect_within(
            carriage,
            rail,
            axes="y",
            margin=0.005,
            name="carriage centered on rail width (y-axis)",
        )
        
        # Check carriage sits on top of rail body (contact along Z)
        # Use specific elements to avoid interference from side lips
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            min_gap=0.0,
            max_gap=0.005,
            positive_elem="carriage_block",
            negative_elem="rail_body",
            name="carriage sits on rail surface",
        )

        # Check that carriage is within the side lips (captured)
        # Carriage half-width = 0.035, lip inner face at y = 0.042
        # So there's 0.007m clearance on each side
        ctx.expect_within(
            carriage,
            rail,
            axes="y",
            margin=0.007,
            name="carriage captured within side lips",
        )

    # Check extended positions - carriage moves along +X
    with ctx.pose({slider: 0.22}):
        pos_right = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            rail,
            axes="y",
            margin=0.005,
            name="carriage stays centered when extended right",
        )

    # Check extended positions - carriage moves along -X
    with ctx.pose({slider: -0.22}):
        pos_left = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            rail,
            axes="y",
            margin=0.005,
            name="carriage stays centered when extended left",
        )

    # Verify the slider actually moves along X axis
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slider: 0.22}):
        extended_pos = ctx.part_world_position(carriage)
    
    ctx.check(
        "carriage slides along +X axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.1,
        details=f"rest={rest_pos}, extended_right={extended_pos}",
    )

    # Check travel range
    with ctx.pose({slider: -0.22}):
        left_pos = ctx.part_world_position(carriage)
    
    ctx.check(
        "carriage slides along -X axis",
        rest_pos is not None
        and left_pos is not None
        and left_pos[0] < rest_pos[0] - 0.1,
        details=f"rest={rest_pos}, extended_left={left_pos}",
    )

    return ctx.report()


object_model = build_object_model()
