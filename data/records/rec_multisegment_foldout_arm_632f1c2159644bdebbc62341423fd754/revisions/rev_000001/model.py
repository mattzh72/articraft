from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LINK_1_LENGTH = 0.150
LINK_2_LENGTH = 0.122
LINK_3_LENGTH = 0.098

LINK_1_BARREL_LEN = 0.030
LINK_2_BARREL_LEN = 0.026
LINK_3_BARREL_LEN = 0.022
PLATFORM_BARREL_LEN = 0.018


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _extrude_profile_y(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width / 2.0, both=True)


def _fuse_all(solids: list[cq.Workplane]) -> cq.Workplane:
    shape = solids[0]
    for solid in solids[1:]:
        shape = shape.union(solid)
    return shape


def _bridge_support_shape() -> cq.Workplane:
    support = _extrude_profile_y(
        [
            (-0.092, 0.000),
            (-0.092, 0.014),
            (-0.071, 0.014),
            (-0.064, 0.067),
            (-0.046, 0.089),
            (-0.012, 0.095),
            (0.000, 0.036),
            (0.000, -0.018),
            (-0.020, -0.018),
            (-0.040, -0.005),
            (-0.076, 0.000),
        ],
        0.104,
    )
    support = support.cut(_box((0.020, LINK_1_BARREL_LEN + 0.012, 0.028), (-0.010, 0.0, 0.0)))

    for x_pos in (-0.072, -0.046):
        for y_pos in (-0.032, 0.032):
            support = support.cut(
                cq.Workplane("XY")
                .circle(0.0045)
                .extrude(0.020, both=True)
                .translate((x_pos, y_pos, 0.007))
            )

    return support


def _link_shape(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    root_barrel_radius: float,
    root_barrel_length: float,
    distal_gap: float,
    ear_thickness: float,
    rail_height_scale: float = 0.92,
) -> cq.Workplane:
    root_height = max(height * 1.18, root_barrel_radius * 1.55)
    tip_height = height * rail_height_scale
    shell_width = max(width, distal_gap + 2.0 * ear_thickness + 0.004, root_barrel_length + 0.006)

    main_body = _extrude_profile_y(
        [
            (0.000, -root_height / 2.0),
            (0.024, -root_height / 2.0),
            (length - 0.028, -tip_height / 2.0),
            (length, -tip_height * 0.34),
            (length, tip_height * 0.34),
            (length - 0.028, tip_height / 2.0),
            (0.024, root_height / 2.0),
            (0.000, root_height / 2.0),
        ],
        shell_width,
    )
    shape = _fuse_all(
        [
            main_body,
            _box((0.022, shell_width * 0.84, root_height * 1.02), (0.011, 0.0, 0.0)),
            _box((0.022, shell_width * 0.72, tip_height * 0.96), (length - 0.011, 0.0, 0.0)),
        ]
    )
    window_height = max(height - 2.0 * wall, height * 0.38)
    window_1 = _box((max(length * 0.22, 0.024), shell_width + 0.006, window_height), (length * 0.34, 0.0, 0.0))
    window_2 = _box((max(length * 0.16, 0.020), shell_width + 0.006, window_height * 0.88), (length * 0.63, 0.0, 0.0))
    shape = shape.cut(window_1).cut(window_2)
    shape = shape.cut(_box((0.024, distal_gap, tip_height * 0.88), (length - 0.012, 0.0, 0.0)))
    return shape


def _platform_bracket_shape() -> cq.Workplane:
    bracket = _extrude_profile_y(
        [
            (0.000, -0.004),
            (0.000, 0.012),
            (0.018, 0.012),
            (0.030, 0.006),
            (0.084, 0.006),
            (0.084, -0.014),
            (0.026, -0.014),
            (0.014, -0.004),
        ],
        0.042,
    )
    bracket = bracket.cut(_box((0.032, 0.024, 0.004), (0.056, 0.0, -0.010)))
    for x_pos in (0.046, 0.066):
        for y_pos in (-0.010, 0.010):
            bracket = bracket.cut(
                cq.Workplane("XY")
                .circle(0.0030)
                .extrude(0.012, both=True)
                .translate((x_pos, y_pos, -0.010))
            )
    return bracket


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=visual_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_foldout_arm")

    model.material("powder_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("machined_aluminum", rgba=(0.69, 0.72, 0.76, 1.0))
    model.material("anodized_aluminum", rgba=(0.58, 0.62, 0.67, 1.0))
    model.material("dark_anodized", rgba=(0.27, 0.29, 0.32, 1.0))

    bridge_support = model.part("bridge_support")
    link_1 = model.part("link_1")
    link_2 = model.part("link_2")
    link_3 = model.part("link_3")
    platform_bracket = model.part("platform_bracket")

    _add_mesh_visual(
        bridge_support,
        _bridge_support_shape(),
        "bridge_support",
        "powder_steel",
        "support_shell",
    )
    _add_mesh_visual(
        link_1,
        _link_shape(
            length=LINK_1_LENGTH,
            width=0.042,
            height=0.030,
            wall=0.0055,
            root_barrel_radius=0.016,
            root_barrel_length=LINK_1_BARREL_LEN,
            distal_gap=LINK_2_BARREL_LEN,
            ear_thickness=0.007,
        ),
        "link_1",
        "machined_aluminum",
        "link_1_shell",
    )
    _add_mesh_visual(
        link_2,
        _link_shape(
            length=LINK_2_LENGTH,
            width=0.034,
            height=0.025,
            wall=0.0048,
            root_barrel_radius=0.014,
            root_barrel_length=LINK_2_BARREL_LEN,
            distal_gap=LINK_3_BARREL_LEN,
            ear_thickness=0.006,
        ),
        "link_2",
        "anodized_aluminum",
        "link_2_shell",
    )
    _add_mesh_visual(
        link_3,
        _link_shape(
            length=LINK_3_LENGTH,
            width=0.026,
            height=0.020,
            wall=0.0038,
            root_barrel_radius=0.012,
            root_barrel_length=LINK_3_BARREL_LEN,
            distal_gap=PLATFORM_BARREL_LEN,
            ear_thickness=0.005,
            rail_height_scale=0.86,
        ),
        "link_3",
        "dark_anodized",
        "link_3_shell",
    )
    _add_mesh_visual(
        platform_bracket,
        _platform_bracket_shape(),
        "platform_bracket",
        "machined_aluminum",
        "platform_shell",
    )

    model.articulation(
        "support_to_link_1",
        ArticulationType.REVOLUTE,
        parent=bridge_support,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.85, upper=1.10),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=1.6, lower=-2.10, upper=2.10),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=-2.20, upper=2.20),
    )
    model.articulation(
        "link_3_to_platform",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=platform_bracket,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge_support = object_model.get_part("bridge_support")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    platform_bracket = object_model.get_part("platform_bracket")

    support_to_link_1 = object_model.get_articulation("support_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_platform = object_model.get_articulation("link_3_to_platform")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.expect_contact(bridge_support, link_1, contact_tol=0.0005, name="support root hinge is carried by clevis")
    ctx.expect_contact(link_1, link_2, contact_tol=0.0005, name="link_1 carries link_2 at elbow hinge")
    ctx.expect_contact(link_2, link_3, contact_tol=0.0005, name="link_2 carries link_3 at elbow hinge")
    ctx.expect_contact(link_3, platform_bracket, contact_tol=0.0005, name="link_3 carries platform bracket at wrist hinge")

    rest_link_2 = None
    rest_link_3 = None
    rest_platform = None
    rest_platform_aabb = None
    with ctx.pose(
        {
            support_to_link_1: 0.0,
            link_1_to_link_2: 0.0,
            link_2_to_link_3: 0.0,
            link_3_to_platform: 0.0,
        }
    ):
        pos_1 = ctx.part_world_position(link_1)
        pos_2 = ctx.part_world_position(link_2)
        pos_3 = ctx.part_world_position(link_3)
        pos_4 = ctx.part_world_position(platform_bracket)
        aabb_4 = ctx.part_world_aabb(platform_bracket)
        rest_link_2 = pos_2
        rest_link_3 = pos_3
        rest_platform = pos_4
        rest_platform_aabb = aabb_4
        straight_ok = (
            pos_1 is not None
            and pos_2 is not None
            and pos_3 is not None
            and pos_4 is not None
            and abs(pos_1[0]) < 0.002
            and abs(pos_1[2]) < 0.002
            and abs(pos_2[0] - LINK_1_LENGTH) < 0.002
            and abs(pos_2[2]) < 0.002
            and abs(pos_3[0] - (LINK_1_LENGTH + LINK_2_LENGTH)) < 0.002
            and abs(pos_3[2]) < 0.002
            and abs(pos_4[0] - (LINK_1_LENGTH + LINK_2_LENGTH + LINK_3_LENGTH)) < 0.002
            and abs(pos_4[2]) < 0.002
        )
        ctx.check(
            "straight pose establishes the tapered four-joint chain",
            straight_ok,
            details=f"positions: link_1={pos_1}, link_2={pos_2}, link_3={pos_3}, platform={pos_4}",
        )

    with ctx.pose({link_1_to_link_2: 0.70}):
        raised_link_3 = ctx.part_world_position(link_3)
        elbow_lifts = (
            rest_link_3 is not None
            and raised_link_3 is not None
            and raised_link_3[2] > rest_link_3[2] + 0.060
            and raised_link_3[0] < rest_link_3[0] - 0.020
        )
        ctx.check(
            "positive second-joint rotation folds the distal chain upward",
            elbow_lifts,
            details=f"rest_link_3={rest_link_3}, raised_link_3={raised_link_3}",
        )

    with ctx.pose({link_3_to_platform: 0.70}):
        raised_platform = ctx.part_world_position(platform_bracket)
        raised_platform_aabb = ctx.part_world_aabb(platform_bracket)
        rest_center_z = None
        raised_center_z = None
        if rest_platform_aabb is not None and raised_platform_aabb is not None:
            rest_center_z = (rest_platform_aabb[0][2] + rest_platform_aabb[1][2]) / 2.0
            raised_center_z = (raised_platform_aabb[0][2] + raised_platform_aabb[1][2]) / 2.0
        wrist_lifts = (
            rest_platform is not None
            and raised_platform is not None
            and abs(raised_platform[0] - rest_platform[0]) < 0.001
            and rest_center_z is not None
            and raised_center_z is not None
            and raised_center_z > rest_center_z + 0.020
        )
        ctx.check(
            "positive wrist rotation tips the small platform bracket upward",
            wrist_lifts,
            details=(
                f"rest_platform={rest_platform}, raised_platform={raised_platform}, "
                f"rest_aabb={rest_platform_aabb}, raised_aabb={raised_platform_aabb}, "
                f"rest_center_z={rest_center_z}, raised_center_z={raised_center_z}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
