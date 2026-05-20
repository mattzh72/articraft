from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(profile, *, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _build_valve_body(model: ArticulatedObject, name: str, chrome, dark_metal):
    body = model.part(name)
    body.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=chrome,
        name="valve_shank",
    )
    body.visual(
        Cylinder(radius=0.0155, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=dark_metal,
        name="mounting_nut",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=chrome,
        name="escutcheon",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=chrome,
        name="bonnet",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.090)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )
    return body


def _build_handle(model: ArticulatedObject, name: str, chrome, accent):
    handle = model.part(name)
    lever_geom = ExtrudeGeometry(
        rounded_rect_profile(0.018, 0.010, 0.0035, corner_segments=6),
        0.064,
        center=True,
    ).rotate_x(pi / 2.0)
    handle.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=chrome,
        name="handle_skirt",
    )
    handle.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=chrome,
        name="handle_hub",
    )
    handle.visual(
        _mesh(f"{name}_handle_lever", lever_geom),
        origin=Origin(xyz=(0.0, -0.032, 0.032)),
        material=chrome,
        name="handle_lever",
    )
    handle.visual(
        Cylinder(radius=0.0085, length=0.018),
        origin=Origin(xyz=(0.0, -0.063, 0.032), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lever_tip",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0305)),
        material=accent,
        name="temperature_index",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.030, 0.082, 0.040)),
        mass=0.20,
        origin=Origin(xyz=(0.0, -0.025, 0.020)),
    )
    return handle


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_bathroom_faucet")

    porcelain = model.material("porcelain", rgba=(0.95, 0.96, 0.97, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.89, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.19, 0.15, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.18, 0.43, 0.84, 1.0))

    deck_width = 0.42
    deck_depth = 0.16
    deck_thickness = 0.018
    handle_spacing = 0.26
    mount_row_y = 0.028
    hole_radius = 0.018

    deck = model.part("deck")
    deck_profile = rounded_rect_profile(deck_width, deck_depth, 0.018, corner_segments=8)
    hole_profile = superellipse_profile(hole_radius * 2.0, hole_radius * 2.0, exponent=2.0, segments=30)
    deck_mesh = ExtrudeWithHolesGeometry(
        deck_profile,
        [
            _translate_profile(hole_profile, dx=-handle_spacing * 0.5, dy=mount_row_y),
            _translate_profile(hole_profile, dx=0.0, dy=mount_row_y),
            _translate_profile(hole_profile, dx=handle_spacing * 0.5, dy=mount_row_y),
        ],
        deck_thickness,
        center=True,
    )
    deck.visual(
        _mesh("deck_shell", deck_mesh),
        origin=Origin(xyz=(0.0, 0.0, -deck_thickness * 0.5)),
        material=porcelain,
        name="deck_shell",
    )
    deck.inertial = Inertial.from_geometry(
        Box((deck_width, deck_depth, deck_thickness)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, -deck_thickness * 0.5)),
    )

    spout_base = model.part("spout_base")
    spout_base.visual(
        Cylinder(radius=0.011, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=chrome,
        name="spout_shank",
    )
    spout_base.visual(
        Cylinder(radius=0.0175, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=dark_metal,
        name="spout_nut",
    )
    spout_base.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=chrome,
        name="spout_escutcheon",
    )
    spout_base.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=chrome,
        name="bearing_housing",
    )
    spout_base.inertial = Inertial.from_geometry(
        Box((0.064, 0.064, 0.094)),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.016, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=chrome,
        name="spout_sleeve",
    )
    spout.visual(
        _mesh(
            "spout_neck",
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.044),
                    (0.0, -0.004, 0.106),
                    (0.0, -0.035, 0.159),
                    (0.0, -0.088, 0.181),
                    (0.0, -0.118, 0.152),
                    (0.0, -0.120, 0.129),
                ],
                radius=0.0115,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=False,
            ),
        ),
        material=chrome,
        name="spout_neck",
    )
    spout.visual(
        _mesh(
            "spout_mouth",
            tube_from_spline_points(
                [
                    (0.0, -0.119, 0.133),
                    (0.0, -0.127, 0.112),
                    (0.0, -0.129, 0.094),
                ],
                radius=0.0115,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=False,
            ),
        ),
        material=chrome,
        name="spout_mouth",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.040, 0.140, 0.190)),
        mass=0.80,
        origin=Origin(xyz=(0.0, -0.060, 0.095)),
    )

    hot_body = _build_valve_body(model, "hot_valve_body", chrome, dark_metal)
    cold_body = _build_valve_body(model, "cold_valve_body", chrome, dark_metal)
    hot_handle = _build_handle(model, "hot_handle", chrome, hot_red)
    cold_handle = _build_handle(model, "cold_handle", chrome, cold_blue)

    model.articulation(
        "deck_to_spout_base",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_base,
        origin=Origin(xyz=(0.0, mount_row_y, 0.0)),
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=spout_base,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0),
    )
    model.articulation(
        "deck_to_hot_body",
        ArticulationType.FIXED,
        parent=deck,
        child=hot_body,
        origin=Origin(xyz=(-handle_spacing * 0.5, mount_row_y, 0.0)),
    )
    model.articulation(
        "deck_to_cold_body",
        ArticulationType.FIXED,
        parent=deck,
        child=cold_body,
        origin=Origin(xyz=(handle_spacing * 0.5, mount_row_y, 0.0)),
    )
    model.articulation(
        "hot_handle_turn",
        ArticulationType.REVOLUTE,
        parent=hot_body,
        child=hot_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.90, upper=0.90),
    )
    model.articulation(
        "cold_handle_turn",
        ArticulationType.REVOLUTE,
        parent=cold_body,
        child=cold_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.90, upper=0.90),
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

    deck = object_model.get_part("deck")
    spout_base = object_model.get_part("spout_base")
    spout = object_model.get_part("spout")
    hot_body = object_model.get_part("hot_valve_body")
    cold_body = object_model.get_part("cold_valve_body")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_handle_turn")
    cold_turn = object_model.get_articulation("cold_handle_turn")

    ctx.check(
        "spout uses continuous vertical swivel",
        spout_swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={spout_swivel.articulation_type}, axis={spout_swivel.axis}",
    )
    ctx.check(
        "handles use vertical revolute stems",
        hot_turn.articulation_type == ArticulationType.REVOLUTE
        and cold_turn.articulation_type == ArticulationType.REVOLUTE
        and tuple(hot_turn.axis) == (0.0, 0.0, -1.0)
        and tuple(cold_turn.axis) == (0.0, 0.0, 1.0),
        details=f"hot={hot_turn.axis}, cold={cold_turn.axis}",
    )

    ctx.expect_gap(
        spout_base,
        deck,
        axis="z",
        positive_elem="spout_escutcheon",
        negative_elem="deck_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="spout escutcheon seats on deck",
    )
    ctx.expect_gap(
        hot_body,
        deck,
        axis="z",
        positive_elem="escutcheon",
        negative_elem="deck_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="hot escutcheon seats on deck",
    )
    ctx.expect_gap(
        cold_body,
        deck,
        axis="z",
        positive_elem="escutcheon",
        negative_elem="deck_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="cold escutcheon seats on deck",
    )

    ctx.expect_origin_distance(
        spout_base,
        spout,
        axes="xy",
        max_dist=0.001,
        name="spout remains centered on swivel post",
    )
    ctx.expect_origin_distance(
        hot_body,
        hot_handle,
        axes="xy",
        max_dist=0.001,
        name="hot handle stays centered on valve stem",
    )
    ctx.expect_origin_distance(
        cold_body,
        cold_handle,
        axes="xy",
        max_dist=0.001,
        name="cold handle stays centered on valve stem",
    )
    ctx.expect_origin_gap(
        cold_body,
        hot_body,
        axis="x",
        min_gap=0.25,
        max_gap=0.27,
        name="hot and cold valves are widely spaced",
    )

    rest_spout_mouth = _aabb_center(ctx.part_element_world_aabb(spout, elem="spout_mouth"))
    with ctx.pose({spout_swivel: pi / 2.0}):
        turned_spout_mouth = _aabb_center(ctx.part_element_world_aabb(spout, elem="spout_mouth"))
    ctx.check(
        "spout mouth swings around the center post",
        rest_spout_mouth is not None
        and turned_spout_mouth is not None
        and abs(rest_spout_mouth[0]) < 0.02
        and turned_spout_mouth[0] > rest_spout_mouth[0] + 0.08
        and turned_spout_mouth[1] > rest_spout_mouth[1] + 0.08,
        details=f"rest={rest_spout_mouth}, turned={turned_spout_mouth}",
    )

    rest_hot_lever = _aabb_center(ctx.part_element_world_aabb(hot_handle, elem="handle_lever"))
    with ctx.pose({hot_turn: 0.75}):
        turned_hot_lever = _aabb_center(ctx.part_element_world_aabb(hot_handle, elem="handle_lever"))
    ctx.check(
        "hot lever rotates about its stem",
        rest_hot_lever is not None
        and turned_hot_lever is not None
        and turned_hot_lever[0] < rest_hot_lever[0] - 0.015,
        details=f"rest={rest_hot_lever}, turned={turned_hot_lever}",
    )

    rest_cold_lever = _aabb_center(ctx.part_element_world_aabb(cold_handle, elem="handle_lever"))
    with ctx.pose({cold_turn: 0.75}):
        turned_cold_lever = _aabb_center(ctx.part_element_world_aabb(cold_handle, elem="handle_lever"))
    ctx.check(
        "cold lever rotates about its stem",
        rest_cold_lever is not None
        and turned_cold_lever is not None
        and turned_cold_lever[0] > rest_cold_lever[0] + 0.015,
        details=f"rest={rest_cold_lever}, turned={turned_cold_lever}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
