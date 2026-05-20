from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

HINGE_HEIGHT = 0.180
LEAF_WIDTH = 0.085
PLATE_THICKNESS = 0.014
PLATE_CENTER_X = 0.0705
BARREL_OUTER_R = 0.018
LEAF_BORE_R = 0.0125
BUSHING_OUTER_R = 0.0117
BUSHING_FLANGE_R = 0.0123
BUSHING_INNER_R = 0.0108
PIN_RADIUS = 0.0105
TOP_SEG_LEN = 0.052
MID_SEG_LEN = 0.070
BOTTOM_SEG_LEN = 0.052
TOP_SEG_CENTER_Z = 0.064
BOTTOM_SEG_CENTER_Z = -0.064
OPEN_LIMIT = 1.90


def _ring_segment(length: float, outer_r: float, inner_r: float, z_center: float) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(outer_r)
        .extrude(length)
        .translate((0.0, 0.0, z_center - length / 2.0))
    )
    inner = (
        cq.Workplane("XY")
        .circle(inner_r)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, z_center - length / 2.0 - 0.002))
    )
    return outer.cut(inner)


def _solid_segment(length: float, radius: float, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, z_center - length / 2.0))
    )


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _hex_prism(diameter: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").polygon(6, diameter).extrude(height).translate(center)


def _round_pad(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(height).translate(center)


def _leaf_bolts(plate_center_x: float, y_sign: float) -> cq.Workplane:
    bolt_positions = [
        (plate_center_x - 0.022, -0.055),
        (plate_center_x + 0.022, -0.055),
        (plate_center_x - 0.022, 0.055),
        (plate_center_x + 0.022, 0.055),
    ]
    solids: list[cq.Workplane] = []
    for x_pos, z_pos in bolt_positions:
        head_y = y_sign * (PLATE_THICKNESS / 2.0)
        solids.append(_round_pad(0.0065, 0.0015, (x_pos, head_y, z_pos)))
        solids.append(_hex_prism(0.0115, 0.004, (x_pos, head_y + y_sign * 0.0015, z_pos)))
        solids.append(_round_pad(0.0034, 0.006, (x_pos, head_y - y_sign * 0.0045, z_pos)))
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _cover_bolts(plate_center_x: float, y_face: float, x_span: float, z_span: float) -> cq.Workplane:
    bolt_positions = [
        (plate_center_x - x_span / 2.0, -z_span / 2.0),
        (plate_center_x + x_span / 2.0, -z_span / 2.0),
        (plate_center_x - x_span / 2.0, z_span / 2.0),
        (plate_center_x + x_span / 2.0, z_span / 2.0),
    ]
    solids: list[cq.Workplane] = []
    for x_pos, z_pos in bolt_positions:
        solids.append(_round_pad(0.0044, 0.0025, (x_pos, y_face, z_pos)))
        solids.append(_round_pad(0.0020, 0.0025, (x_pos, y_face - 0.00125 if y_face < 0.0 else y_face + 0.00125, z_pos)))
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _base_leaf_shape() -> cq.Workplane:
    plate = _box(LEAF_WIDTH, PLATE_THICKNESS, HINGE_HEIGHT, (-PLATE_CENTER_X, 0.0, 0.0))
    top_barrel = _ring_segment(TOP_SEG_LEN, BARREL_OUTER_R, LEAF_BORE_R, TOP_SEG_CENTER_Z)
    bottom_barrel = _ring_segment(BOTTOM_SEG_LEN, BARREL_OUTER_R, LEAF_BORE_R, BOTTOM_SEG_CENTER_Z)

    top_cheek = _box(0.032, 0.014, 0.044, (-0.016, -0.008, TOP_SEG_CENTER_Z))
    bottom_cheek = _box(0.032, 0.014, 0.044, (-0.016, -0.008, BOTTOM_SEG_CENTER_Z))
    mid_bridge = _box(0.020, 0.012, 0.028, (-0.022, -0.008, 0.0))
    outer_rib = _box(0.014, 0.008, 0.106, (-0.041, -0.011, 0.0))
    inner_rib = _box(0.014, 0.006, 0.106, (-0.041, -0.003, 0.0))
    stop_lug = _box(0.018, 0.008, 0.022, (-0.019, 0.012, 0.0))
    access_pad = _box(0.058, 0.004, 0.082, (-PLATE_CENTER_X, -0.009, 0.0))

    leaf = (
        plate.union(top_barrel)
        .union(bottom_barrel)
        .union(top_cheek)
        .union(bottom_cheek)
        .union(mid_bridge)
        .union(outer_rib)
        .union(inner_rib)
        .union(stop_lug)
        .union(access_pad)
        .union(_leaf_bolts(-PLATE_CENTER_X, -1.0))
    )

    center_relief = _box(0.030, 0.024, 0.040, (-0.010, 0.0, 0.0))
    hinge_bore = _solid_segment(HINGE_HEIGHT + 0.008, 0.0128, 0.0)
    leaf = leaf.cut(center_relief).cut(hinge_bore)
    return leaf


def _swing_leaf_shape() -> cq.Workplane:
    plate = _box(LEAF_WIDTH, PLATE_THICKNESS, HINGE_HEIGHT, (PLATE_CENTER_X, 0.0, 0.0))
    mid_barrel = _ring_segment(MID_SEG_LEN, BARREL_OUTER_R, LEAF_BORE_R, 0.0)
    cheek = _box(0.034, 0.014, 0.080, (0.017, 0.008, 0.0))
    inner_web = _box(0.022, 0.012, 0.118, (0.032, 0.008, 0.0))
    side_rib_a = _box(0.016, 0.006, 0.112, (0.047, 0.003, 0.0))
    side_rib_b = _box(0.016, 0.008, 0.112, (0.047, 0.011, 0.0))
    stop_lug = _box(0.016, 0.008, 0.020, (0.017, -0.012, 0.010))
    access_pad = _box(0.052, 0.004, 0.070, (PLATE_CENTER_X, 0.009, 0.0))

    leaf = (
        plate.union(mid_barrel)
        .union(cheek)
        .union(inner_web)
        .union(side_rib_a)
        .union(side_rib_b)
        .union(stop_lug)
        .union(access_pad)
        .union(_leaf_bolts(PLATE_CENTER_X, 1.0))
    )

    center_relief = _box(0.024, 0.024, 0.052, (0.012, 0.0, 0.0))
    hinge_bore = _solid_segment(HINGE_HEIGHT + 0.008, 0.0128, 0.0)
    leaf = leaf.cut(center_relief).cut(hinge_bore)
    return leaf.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0)


def _base_bushing_shape() -> cq.Workplane:
    top = _ring_segment(TOP_SEG_LEN - 0.006, BUSHING_OUTER_R, BUSHING_INNER_R, TOP_SEG_CENTER_Z)
    bottom = _ring_segment(BOTTOM_SEG_LEN - 0.006, BUSHING_OUTER_R, BUSHING_INNER_R, BOTTOM_SEG_CENTER_Z)
    top_flange = _ring_segment(0.003, BUSHING_FLANGE_R, BUSHING_INNER_R, TOP_SEG_CENTER_Z - TOP_SEG_LEN / 2.0 + 0.0015)
    bottom_flange = _ring_segment(0.003, BUSHING_FLANGE_R, BUSHING_INNER_R, BOTTOM_SEG_CENTER_Z + BOTTOM_SEG_LEN / 2.0 - 0.0015)
    return top.union(bottom).union(top_flange).union(bottom_flange)


def _swing_bushing_shape() -> cq.Workplane:
    sleeve = _ring_segment(MID_SEG_LEN - 0.006, BUSHING_OUTER_R, BUSHING_INNER_R, 0.0)
    top_flange = _ring_segment(0.003, BUSHING_FLANGE_R, BUSHING_INNER_R, MID_SEG_LEN / 2.0 - 0.0015)
    bottom_flange = _ring_segment(0.003, BUSHING_FLANGE_R, BUSHING_INNER_R, -MID_SEG_LEN / 2.0 + 0.0015)
    return sleeve.union(top_flange).union(bottom_flange)


def _pin_shape() -> cq.Workplane:
    shaft = _solid_segment(0.196, PIN_RADIUS, 0.0)
    top_collar = _solid_segment(0.006, 0.015, 0.093)
    bottom_collar = _solid_segment(0.006, 0.015, -0.093)
    dome = (
        cq.Workplane("XY")
        .circle(0.0125)
        .workplane(offset=0.008)
        .circle(0.0065)
        .loft()
        .translate((0.0, 0.0, 0.104))
    )
    nipple_hex = (
        cq.Workplane("XY")
        .polygon(6, 0.008)
        .extrude(0.004)
        .translate((0.0, 0.0, 0.112))
    )
    nipple_nozzle = _solid_segment(0.006, 0.0022, 0.117)
    bottom_head = (
        cq.Workplane("XY")
        .polygon(6, 0.010)
        .extrude(0.004)
        .translate((0.0, 0.0, -0.111))
    )
    return shaft.union(top_collar).union(bottom_collar).union(dome).union(nipple_hex).union(nipple_nozzle).union(bottom_head)


def _base_cover_shape() -> cq.Workplane:
    cover = _box(0.050, 0.004, 0.074, (-PLATE_CENTER_X, 0.010, 0.0))
    boss = _round_pad(0.009, 0.004, (-PLATE_CENTER_X, 0.014, 0.0))
    return cover.union(boss).union(_cover_bolts(-PLATE_CENTER_X, 0.012, 0.032, 0.052))


def _swing_cover_shape() -> cq.Workplane:
    cover = _box(0.046, 0.004, 0.064, (PLATE_CENTER_X, -0.010, 0.0))
    boss = _round_pad(0.008, 0.0035, (PLATE_CENTER_X, -0.0135, 0.0))
    shape = cover.union(boss).union(_cover_bolts(PLATE_CENTER_X, -0.012, 0.030, 0.044))
    return shape.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_hinge_study", assets=ASSETS)

    steel = model.material("steel", rgba=(0.34, 0.36, 0.39, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.20, 0.22, 0.24, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.46, 0.48, 0.50, 1.0))
    pin_metal = model.material("pin_metal", rgba=(0.60, 0.62, 0.66, 1.0))
    bronze = model.material("bronze", rgba=(0.65, 0.48, 0.24, 1.0))

    base_leaf = model.part("base_leaf")
    base_leaf.visual(
        mesh_from_cadquery(_base_leaf_shape(), "base_leaf.obj", assets=ASSETS),
        material=steel,
        name="base_leaf_body",
    )

    swing_leaf = model.part("swing_leaf")
    swing_leaf.visual(
        mesh_from_cadquery(_swing_leaf_shape(), "swing_leaf.obj", assets=ASSETS),
        material=dark_oxide,
        name="swing_leaf_body",
    )

    hinge_pin = model.part("hinge_pin")
    hinge_pin.visual(
        mesh_from_cadquery(_pin_shape(), "hinge_pin.obj", assets=ASSETS),
        material=pin_metal,
        name="hinge_pin_body",
    )

    base_bushings = model.part("base_bushings")
    base_bushings.visual(
        mesh_from_cadquery(_base_bushing_shape(), "base_bushings.obj", assets=ASSETS),
        material=bronze,
        name="base_bushing_body",
    )

    swing_bushing = model.part("swing_bushing")
    swing_bushing.visual(
        mesh_from_cadquery(_swing_bushing_shape(), "swing_bushing.obj", assets=ASSETS),
        material=bronze,
        name="swing_bushing_body",
    )

    base_cover = model.part("base_cover")
    base_cover.visual(
        mesh_from_cadquery(_base_cover_shape(), "base_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="base_cover_body",
    )

    swing_cover = model.part("swing_cover")
    swing_cover.visual(
        mesh_from_cadquery(_swing_cover_shape(), "swing_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="swing_cover_body",
    )

    model.articulation(
        "base_to_swing",
        ArticulationType.REVOLUTE,
        parent=base_leaf,
        child=swing_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=0.0, upper=OPEN_LIMIT),
    )

    model.articulation(
        "base_to_pin",
        ArticulationType.FIXED,
        parent=base_leaf,
        child=hinge_pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_base_bushings",
        ArticulationType.FIXED,
        parent=base_leaf,
        child=base_bushings,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_base_cover",
        ArticulationType.FIXED,
        parent=base_leaf,
        child=base_cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "swing_to_bushing",
        ArticulationType.FIXED,
        parent=swing_leaf,
        child=swing_bushing,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "swing_to_cover",
        ArticulationType.FIXED,
        parent=swing_leaf,
        child=swing_cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_leaf = object_model.get_part("base_leaf")
    swing_leaf = object_model.get_part("swing_leaf")
    hinge_pin = object_model.get_part("hinge_pin")
    base_bushings = object_model.get_part("base_bushings")
    swing_bushing = object_model.get_part("swing_bushing")
    base_cover = object_model.get_part("base_cover")
    swing_cover = object_model.get_part("swing_cover")
    hinge = object_model.get_articulation("base_to_swing")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=10,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(hinge_pin, base_leaf, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(base_bushings, base_leaf, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(swing_bushing, swing_leaf, axes="xy", max_dist=0.001)

    ctx.expect_overlap(hinge_pin, base_leaf, axes="z", min_overlap=0.10)
    ctx.expect_overlap(hinge_pin, swing_leaf, axes="z", min_overlap=0.06)
    ctx.expect_overlap(base_bushings, base_leaf, axes="z", min_overlap=0.09)
    ctx.expect_overlap(swing_bushing, swing_leaf, axes="z", min_overlap=0.06)
    ctx.expect_overlap(base_cover, base_leaf, axes="xz", min_overlap=0.040)
    ctx.expect_overlap(swing_cover, swing_leaf, axes="xz", min_overlap=0.036)
    ctx.expect_overlap(swing_leaf, base_leaf, axes="z", min_overlap=0.16)

    ctx.expect_gap(
        base_leaf,
        base_cover,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        swing_cover,
        swing_leaf,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
    )

    rest_box = ctx.part_element_world_aabb(swing_leaf, elem="swing_leaf_body")
    with ctx.pose({hinge: 1.25}):
        ctx.expect_overlap(swing_leaf, base_leaf, axes="z", min_overlap=0.16)
        ctx.expect_overlap(swing_cover, swing_leaf, axes="xz", min_overlap=0.036)
        open_box = ctx.part_element_world_aabb(swing_leaf, elem="swing_leaf_body")

    if rest_box is None or open_box is None:
        ctx.fail("swing_leaf_pose_measurements_available", "missing swing leaf body AABB in one or more poses")
    else:
        rest_center = tuple((a + b) / 2.0 for a, b in zip(rest_box[0], rest_box[1]))
        open_center = tuple((a + b) / 2.0 for a, b in zip(open_box[0], open_box[1]))
        ctx.check(
            "swing_leaf_rotates_about_pin_axis",
            open_center[1] > 0.040 and open_center[0] < rest_center[0] - 0.020,
            details=(
                f"expected opened swing leaf center to move into +Y and back toward axis; "
                f"rest_center={rest_center}, open_center={open_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
