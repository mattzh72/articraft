from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


DECK_WIDTH = 0.38
DECK_DEPTH = 0.18
DECK_THICKNESS = 0.028
HOLE_RADIUS = 0.018
MOUNT_Y = -0.010
HANDLE_SPREAD = 0.2032
HANDLE_OFFSET = HANDLE_SPREAD * 0.5


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 28,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * idx) / segments),
            cy + radius * math.sin((2.0 * math.pi * idx) / segments),
        )
        for idx in range(segments)
    ]


def _wing_handle_profile() -> list[tuple[float, float]]:
    control_points = [
        (0.057, 0.000),
        (0.046, 0.019),
        (0.020, 0.015),
        (0.008, 0.009),
        (-0.008, 0.009),
        (-0.020, 0.015),
        (-0.046, 0.019),
        (-0.057, 0.000),
        (-0.046, -0.019),
        (-0.020, -0.015),
        (-0.008, -0.009),
        (0.008, -0.009),
        (0.020, -0.015),
        (0.046, -0.019),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=6,
        closed=True,
    )


def _add_valve_base(part, *, metal) -> None:
    part.visual(
        Cylinder(radius=0.0125, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.023), rpy=(0.0, 0.0, 0.0)),
        material=metal,
        name="mount_shank",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=metal,
        name="escutcheon",
    )
    part.visual(
        Cylinder(radius=0.019, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=metal,
        name="bonnet_body",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=metal,
        name="bonnet_cap",
    )


def _add_wing_handle(part, *, metal, accent) -> None:
    wing_mesh = _save_mesh(
        "wing_handle_paddle",
        ExtrudeGeometry(
            _wing_handle_profile(),
            0.008,
            center=False,
        ),
    )
    part.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=metal,
        name="handle_hub",
    )
    part.visual(
        wing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=metal,
        name="wing_paddle",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=accent,
        name="temperature_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_bathroom_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    countertop = model.material("countertop", rgba=(0.95, 0.95, 0.93, 1.0))
    hot_mark = model.material("hot_mark", rgba=(0.84, 0.18, 0.16, 1.0))
    cold_mark = model.material("cold_mark", rgba=(0.19, 0.39, 0.85, 1.0))

    deck = model.part("deck")
    deck_mesh = _save_mesh(
        "faucet_deck",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(DECK_WIDTH, DECK_DEPTH, radius=0.016),
            [
                _circle_profile(HOLE_RADIUS, cx=-HANDLE_OFFSET, cy=MOUNT_Y),
                _circle_profile(HOLE_RADIUS, cx=0.0, cy=MOUNT_Y),
                _circle_profile(HOLE_RADIUS, cx=HANDLE_OFFSET, cy=MOUNT_Y),
            ],
            height=DECK_THICKNESS,
            center=False,
        ),
    )
    deck.visual(deck_mesh, material=countertop, name="deck_slab")

    spout_body = model.part("spout_body")
    spout_body.visual(
        Cylinder(radius=0.0145, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=brushed_steel,
        name="mount_shank",
    )
    spout_body.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=chrome,
        name="spout_escutcheon",
    )
    spout_body.visual(
        Cylinder(radius=0.021, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=chrome,
        name="spout_neck",
    )
    spout_curve = _save_mesh(
        "faucet_spout_curve",
        tube_from_spline_points(
            [
                (0.0, 0.000, 0.044),
                (0.0, 0.028, 0.066),
                (0.0, 0.090, 0.097),
                (0.0, 0.126, 0.084),
            ],
            radius=0.0135,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )
    spout_body.visual(
        spout_curve,
        material=chrome,
        name="spout_curve",
    )
    spout_body.visual(
        Cylinder(radius=0.0125, length=0.028),
        origin=Origin(xyz=(0.0, 0.126, 0.070)),
        material=chrome,
        name="outlet_nozzle",
    )
    spout_body.visual(
        Cylinder(radius=0.0100, length=0.010),
        origin=Origin(xyz=(0.0, 0.126, 0.051)),
        material=brushed_steel,
        name="aerator_tip",
    )

    hot_base = model.part("hot_base")
    _add_valve_base(hot_base, metal=chrome)

    cold_base = model.part("cold_base")
    _add_valve_base(cold_base, metal=chrome)

    hot_handle = model.part("hot_handle")
    _add_wing_handle(hot_handle, metal=chrome, accent=hot_mark)

    cold_handle = model.part("cold_handle")
    _add_wing_handle(cold_handle, metal=chrome, accent=cold_mark)

    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, MOUNT_Y, DECK_THICKNESS)),
    )
    model.articulation(
        "deck_to_hot_base",
        ArticulationType.FIXED,
        parent=deck,
        child=hot_base,
        origin=Origin(xyz=(-HANDLE_OFFSET, MOUNT_Y, DECK_THICKNESS)),
    )
    model.articulation(
        "deck_to_cold_base",
        ArticulationType.FIXED,
        parent=deck,
        child=cold_base,
        origin=Origin(xyz=(HANDLE_OFFSET, MOUNT_Y, DECK_THICKNESS)),
    )
    model.articulation(
        "hot_handle_turn",
        ArticulationType.REVOLUTE,
        parent=hot_base,
        child=hot_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "cold_handle_turn",
        ArticulationType.REVOLUTE,
        parent=cold_base,
        child=cold_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-1.35,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout_body = object_model.get_part("spout_body")
    hot_base = object_model.get_part("hot_base")
    cold_base = object_model.get_part("cold_base")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")

    hot_joint = object_model.get_articulation("hot_handle_turn")
    cold_joint = object_model.get_articulation("cold_handle_turn")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(spout_body, deck, elem_a="spout_escutcheon", elem_b="deck_slab")
    ctx.expect_contact(hot_base, deck, elem_a="escutcheon", elem_b="deck_slab")
    ctx.expect_contact(cold_base, deck, elem_a="escutcheon", elem_b="deck_slab")
    ctx.expect_contact(hot_handle, hot_base, elem_a="handle_hub", elem_b="bonnet_cap")
    ctx.expect_contact(cold_handle, cold_base, elem_a="handle_hub", elem_b="bonnet_cap")

    ctx.expect_overlap(spout_body, deck, axes="xy", elem_a="spout_escutcheon", elem_b="deck_slab", min_overlap=0.06)
    ctx.expect_overlap(hot_base, deck, axes="xy", elem_a="escutcheon", elem_b="deck_slab", min_overlap=0.05)
    ctx.expect_overlap(cold_base, deck, axes="xy", elem_a="escutcheon", elem_b="deck_slab", min_overlap=0.05)

    ctx.expect_origin_distance(hot_base, cold_base, axes="x", min_dist=0.19, max_dist=0.215)
    ctx.expect_origin_distance(spout_body, hot_base, axes="x", min_dist=0.095, max_dist=0.108)
    ctx.expect_origin_distance(spout_body, cold_base, axes="x", min_dist=0.095, max_dist=0.108)

    hot_limits = hot_joint.motion_limits
    cold_limits = cold_joint.motion_limits
    ctx.check(
        "hot_handle_axis_vertical",
        hot_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={hot_joint.axis}",
    )
    ctx.check(
        "cold_handle_axis_vertical",
        cold_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={cold_joint.axis}",
    )
    ctx.check(
        "hot_handle_quarter_turn_range",
        hot_limits is not None
        and hot_limits.lower == 0.0
        and hot_limits.upper is not None
        and 1.1 <= hot_limits.upper <= 1.5,
        details=f"limits={hot_limits}",
    )
    ctx.check(
        "cold_handle_quarter_turn_range",
        cold_limits is not None
        and cold_limits.upper == 0.0
        and cold_limits.lower is not None
        and -1.5 <= cold_limits.lower <= -1.1,
        details=f"limits={cold_limits}",
    )

    outlet_aabb = ctx.part_element_world_aabb(spout_body, elem="aerator_tip")
    spout_aabb = ctx.part_world_aabb(spout_body)
    if outlet_aabb is None:
        ctx.fail("aerator_tip_present", "Could not resolve aerator tip world bounds.")
    else:
        outlet_min, outlet_max = outlet_aabb
        ctx.check(
            "low_arc_outlet_height",
            0.070 <= outlet_min[2] <= 0.085 and 0.080 <= outlet_max[2] <= 0.095,
            details=f"outlet_z=({outlet_min[2]:.4f}, {outlet_max[2]:.4f})",
        )
        ctx.check(
            "spout_reach_forward",
            0.110 <= outlet_max[1] <= 0.145,
            details=f"outlet_y_max={outlet_max[1]:.4f}",
        )
    if spout_aabb is None:
        ctx.fail("spout_body_present", "Could not resolve spout body world bounds.")
    else:
        _, spout_max = spout_aabb
        ctx.check(
            "low_arc_overall_height",
            0.125 <= spout_max[2] <= 0.150,
            details=f"spout_z_max={spout_max[2]:.4f}",
        )

    hot_rest = ctx.part_world_aabb(hot_handle)
    cold_rest = ctx.part_world_aabb(cold_handle)

    if hot_limits is not None and hot_limits.upper is not None:
        with ctx.pose({hot_joint: hot_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hot_handle_open_no_overlap")
            ctx.fail_if_isolated_parts(name="hot_handle_open_no_floating")
            ctx.expect_contact(hot_handle, hot_base, elem_a="handle_hub", elem_b="bonnet_cap", name="hot_handle_open_contact")
            hot_open = ctx.part_world_aabb(hot_handle)
            if hot_rest is not None and hot_open is not None:
                hot_rest_y = hot_rest[1][1] - hot_rest[0][1]
                hot_open_y = hot_open[1][1] - hot_open[0][1]
                ctx.check(
                    "hot_handle_rotates_in_open_pose",
                    hot_open_y > hot_rest_y + 0.020,
                    details=f"rest_y_span={hot_rest_y:.4f}, open_y_span={hot_open_y:.4f}",
                )

    if cold_limits is not None and cold_limits.lower is not None:
        with ctx.pose({cold_joint: cold_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="cold_handle_open_no_overlap")
            ctx.fail_if_isolated_parts(name="cold_handle_open_no_floating")
            ctx.expect_contact(cold_handle, cold_base, elem_a="handle_hub", elem_b="bonnet_cap", name="cold_handle_open_contact")
            cold_open = ctx.part_world_aabb(cold_handle)
            if cold_rest is not None and cold_open is not None:
                cold_rest_y = cold_rest[1][1] - cold_rest[0][1]
                cold_open_y = cold_open[1][1] - cold_open[0][1]
                ctx.check(
                    "cold_handle_rotates_in_open_pose",
                    cold_open_y > cold_rest_y + 0.020,
                    details=f"rest_y_span={cold_rest_y:.4f}, open_y_span={cold_open_y:.4f}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
