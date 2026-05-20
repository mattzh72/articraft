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


PIN_R = 0.0032
PIN_BORE_R = 0.00355
PIN_HEAD_R = 0.0048
PIN_HEAD_W = 0.0020
BARREL_R = 0.0085
CENTER_KNUCKLE_W = 0.0080
SPACER_W = 0.0030
INNER_GAP_W = CENTER_KNUCKLE_W + 2.0 * SPACER_W
CHEEK_W = 0.0070
OUTER_WIDTH = INNER_GAP_W + 2.0 * CHEEK_W
MALE_CENTER_W = 0.0068
MALE_SPACER_W = 0.0022
MALE_STACK_W = MALE_CENTER_W + 2.0 * MALE_SPACER_W
BODY_W = 0.0180
COVER_W = 0.0140


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, center[1], 0.0))
    )


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .center(center[1], center[2])
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((center[0], 0.0, 0.0))
    )


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(center[0], center[1])
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, 0.0, center[2]))
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _segment_body_profile(length: float, top_a: float, top_b: float, bot_a: float, bot_b: float) -> cq.Workplane:
    pts = [
        (0.007, bot_a),
        (0.015, bot_a - 0.004),
        (length * 0.44, bot_a - 0.003),
        (length - 0.016, bot_b - 0.002),
        (length - 0.007, bot_b),
        (length - 0.007, top_b),
        (length - 0.015, top_b + 0.003),
        (0.017, top_a + 0.003),
        (0.006, top_a),
    ]
    return cq.Workplane("XZ").polyline(pts).close().extrude(BODY_W / 2.0, both=True)


def _add_cover_and_fasteners(
    solid: cq.Workplane,
    *,
    x_center: float,
    z_center: float,
    length: float,
) -> cq.Workplane:
    cover = _box((length, COVER_W, 0.0020), (x_center, 0.0, z_center))
    solid = solid.union(cover)
    for sx in (-0.36, 0.36):
        for sy in (-0.33, 0.33):
            solid = solid.union(
                _z_cylinder(
                    0.0014,
                    0.0012,
                    (x_center + sx * length, sy * COVER_W, z_center + 0.0016),
                )
            )
    return solid


def _add_guide_bracket(solid: cq.Workplane, *, x_center: float, z_base: float) -> cq.Workplane:
    solid = solid.union(_box((0.012, 0.010, 0.003), (x_center, 0.0, z_base + 0.0015)))
    for y in (-0.0035, 0.0035):
        solid = solid.union(_box((0.003, 0.003, 0.008), (x_center - 0.001, y, z_base + 0.006)))
    solid = solid.union(_y_cylinder(0.0018, 0.010, (x_center - 0.001, 0.0, z_base + 0.010)))
    return solid


def make_link(
    *,
    length: float,
    top_a: float,
    top_b: float,
    bot_a: float,
    bot_b: float,
    guide_x: float,
    add_tip_cap: bool = False,
) -> cq.Workplane:
    solid = _segment_body_profile(length, top_a, top_b, bot_a, bot_b)
    solid = solid.cut(
        cq.Workplane("XZ")
        .center(length * 0.46, 0.0005)
        .slot2D(length * 0.23, 0.009, angle=0)
        .extrude(0.0055, both=True)
    )

    spacer_center_y = MALE_CENTER_W / 2.0 + MALE_SPACER_W / 2.0
    solid = solid.union(_box((0.014, MALE_STACK_W, 0.016), (0.007, 0.0, 0.0)))
    solid = solid.union(_y_cylinder(BARREL_R * 0.96, MALE_CENTER_W, (0.0, 0.0, 0.0)))
    for y in (-spacer_center_y, spacer_center_y):
        solid = solid.union(_y_cylinder(BARREL_R * 0.84, MALE_SPACER_W, (0.0, y, 0.0)))

    cheek_y = INNER_GAP_W / 2.0 + CHEEK_W / 2.0
    for side in (-1.0, 1.0):
        y_center = side * cheek_y
        solid = solid.union(_box((0.016, CHEEK_W, 0.022), (length - 0.004, y_center, 0.0)))
        solid = solid.union(_y_cylinder(BARREL_R, CHEEK_W, (length, y_center, 0.0)))

    solid = solid.union(_y_cylinder(PIN_R, OUTER_WIDTH, (length, 0.0, 0.0)))
    for y in (-(OUTER_WIDTH / 2.0 + PIN_HEAD_W / 2.0), OUTER_WIDTH / 2.0 + PIN_HEAD_W / 2.0):
        solid = solid.union(_y_cylinder(PIN_HEAD_R, PIN_HEAD_W, (length, y, 0.0)))

    solid = solid.union(_box((0.006, INNER_GAP_W, 0.004), (length - 0.008, 0.0, -0.013)))
    solid = solid.union(_box((0.004, INNER_GAP_W, 0.005), (length - 0.004, 0.0, 0.012)))

    solid = solid.cut(_y_cylinder(PIN_BORE_R, MALE_STACK_W + 0.004, (0.0, 0.0, 0.0)))

    solid = _add_cover_and_fasteners(
        solid,
        x_center=length * 0.47,
        z_center=max(top_a, top_b) + 0.0035,
        length=length * 0.34,
    )
    solid = _add_guide_bracket(solid, x_center=guide_x, z_base=max(top_a, top_b) + 0.0025)

    if add_tip_cap:
        solid = solid.union(_box((0.012, 0.018, 0.014), (length + 0.007, 0.0, 0.0)))
        solid = solid.union(_x_cylinder(0.007, 0.010, (length + 0.004, 0.0, 0.0)))
        solid = solid.union(_box((0.005, 0.014, 0.010), (length + 0.013, 0.0, 0.0)))
        for y in (-0.0045, 0.0045):
            solid = solid.union(_z_cylinder(0.0013, 0.0010, (length + 0.010, y, 0.007)))

    return solid


def make_base_frame() -> cq.Workplane:
    solid = _box((0.096, 0.046, 0.008), (-0.036, 0.0, -0.034))
    slots = (
        cq.Workplane("XY")
        .pushPoints([(-0.055, 0.0), (-0.018, 0.0)])
        .slot2D(0.020, 0.007, angle=0)
        .extrude(0.006, both=True)
        .translate((0.0, 0.0, -0.034))
    )
    solid = solid.cut(slots)

    solid = solid.union(_box((0.026, 0.040, 0.050), (-0.036, 0.0, -0.009)))
    solid = solid.union(_box((0.024, 0.026, 0.020), (-0.018, 0.0, 0.006)))
    solid = solid.union(_box((0.018, INNER_GAP_W, 0.018), (-0.009, 0.0, -0.010)))

    gusset_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.052, -0.030),
                (-0.014, -0.030),
                (-0.014, -0.008),
                (-0.026, 0.012),
                (-0.044, -0.004),
                (-0.052, -0.016),
            ]
        )
        .close()
        .extrude(0.005)
    )
    solid = solid.union(gusset_profile.translate((0.0, 0.013, 0.0)))
    solid = solid.union(gusset_profile.translate((0.0, -0.018, 0.0)))

    cheek_y = INNER_GAP_W / 2.0 + CHEEK_W / 2.0
    for side in (-1.0, 1.0):
        y_center = side * cheek_y
        solid = solid.union(_box((0.018, CHEEK_W, 0.026), (0.0, y_center, 0.0)))
        solid = solid.union(_y_cylinder(BARREL_R, CHEEK_W, (0.0, y_center, 0.0)))

    solid = solid.union(_y_cylinder(PIN_R, OUTER_WIDTH, (0.0, 0.0, 0.0)))
    for y in (-(OUTER_WIDTH / 2.0 + PIN_HEAD_W / 2.0), OUTER_WIDTH / 2.0 + PIN_HEAD_W / 2.0):
        solid = solid.union(_y_cylinder(PIN_HEAD_R, PIN_HEAD_W, (0.0, y, 0.0)))

    solid = solid.union(_box((0.007, INNER_GAP_W, 0.004), (0.011, 0.0, -0.014)))
    solid = solid.union(_box((0.010, 0.003, 0.030), (-0.048, 0.0215, -0.006)))
    for z in (-0.016, 0.005):
        solid = solid.union(_z_cylinder(0.0014, 0.0010, (-0.048, 0.0215, z)))
    solid = solid.union(_box((0.018, 0.020, 0.004), (-0.036, 0.0, 0.017)))
    for x in (-0.042, -0.030):
        solid = solid.union(_z_cylinder(0.0016, 0.0012, (x, -0.006, 0.020)))
        solid = solid.union(_z_cylinder(0.0016, 0.0012, (x, 0.006, 0.020)))

    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_phalanx_chain", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.45, 0.47, 0.50, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(make_base_frame(), "base_frame.obj", assets=ASSETS),
        material=dark_steel,
        origin=Origin(),
        name="structure",
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(
            make_link(
                length=0.060,
                top_a=0.010,
                top_b=0.008,
                bot_a=-0.009,
                bot_b=-0.007,
                guide_x=0.027,
            ),
            "proximal_link.obj",
            assets=ASSETS,
        ),
        material=machined_steel,
        origin=Origin(),
        name="structure",
    )

    middle_link = model.part("middle_link")
    middle_link.visual(
        mesh_from_cadquery(
            make_link(
                length=0.045,
                top_a=0.009,
                top_b=0.007,
                bot_a=-0.008,
                bot_b=-0.006,
                guide_x=0.020,
            ),
            "middle_link.obj",
            assets=ASSETS,
        ),
        material=plate_gray,
        origin=Origin(),
        name="structure",
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(
            make_link(
                length=0.034,
                top_a=0.008,
                top_b=0.006,
                bot_a=-0.007,
                bot_b=-0.005,
                guide_x=0.015,
                add_tip_cap=True,
            ),
            "distal_link.obj",
            assets=ASSETS,
        ),
        material=machined_steel,
        origin=Origin(),
        name="structure",
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=proximal_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.5, lower=-1.10, upper=0.20),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=middle_link,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-1.30, upper=0.15),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=distal_link,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.5, lower=-1.20, upper=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    proximal_link = object_model.get_part("proximal_link")
    middle_link = object_model.get_part("middle_link")
    distal_link = object_model.get_part("distal_link")
    base_to_proximal = object_model.get_articulation("base_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        base_frame,
        proximal_link,
        reason="Nested hinge barrels intentionally interleave at the base pivot.",
    )
    ctx.allow_overlap(
        proximal_link,
        middle_link,
        reason="Nested hinge barrels intentionally interleave at the middle pivot.",
    )
    ctx.allow_overlap(
        middle_link,
        distal_link,
        reason="Nested hinge barrels intentionally interleave at the distal pivot.",
    )

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

    ctx.expect_overlap(proximal_link, base_frame, axes="y", min_overlap=0.012, name="base yoke laterally captures proximal hinge stack")
    ctx.expect_overlap(middle_link, proximal_link, axes="y", min_overlap=0.012, name="proximal yoke laterally captures middle hinge stack")
    ctx.expect_overlap(distal_link, middle_link, axes="y", min_overlap=0.012, name="middle yoke laterally captures distal hinge stack")

    ctx.expect_origin_distance(proximal_link, base_frame, axes="yz", max_dist=0.001, name="proximal joint origin aligns with base hinge axis")
    ctx.expect_origin_distance(middle_link, proximal_link, axes="yz", max_dist=0.001, name="middle joint origin aligns with proximal hinge axis")
    ctx.expect_origin_distance(distal_link, middle_link, axes="yz", max_dist=0.001, name="distal joint origin aligns with middle hinge axis")

    ctx.expect_origin_gap(proximal_link, base_frame, axis="x", min_gap=0.0, max_gap=0.002, name="proximal pivot sits at base joint origin")
    ctx.expect_origin_gap(middle_link, proximal_link, axis="x", min_gap=0.058, max_gap=0.062, name="middle pivot is placed at distal end of proximal link")
    ctx.expect_origin_gap(distal_link, middle_link, axis="x", min_gap=0.043, max_gap=0.047, name="distal pivot is placed at distal end of middle link")

    with ctx.pose(
        {
            base_to_proximal: -0.55,
            proximal_to_middle: -0.72,
            middle_to_distal: -0.52,
        }
    ):
        ctx.expect_overlap(proximal_link, base_frame, axes="y", min_overlap=0.010, name="base hinge remains laterally captured in flexion")
        ctx.expect_overlap(middle_link, proximal_link, axes="y", min_overlap=0.010, name="middle hinge remains laterally captured in flexion")
        ctx.expect_overlap(distal_link, middle_link, axes="y", min_overlap=0.010, name="distal hinge remains laterally captured in flexion")
        ctx.expect_gap(distal_link, base_frame, axis="z", min_gap=0.040, name="distal link rises above the base in flexion")
        ctx.expect_origin_gap(distal_link, base_frame, axis="x", min_gap=0.025, max_gap=0.110, name="distal tip still projects forward in flexion")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
