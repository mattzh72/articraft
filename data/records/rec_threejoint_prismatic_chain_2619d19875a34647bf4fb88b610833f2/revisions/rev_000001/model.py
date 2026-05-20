from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


EXTENSION_AXIS = (0.0, 1.0, 0.0)

OUTER = {
    "width": 0.340,
    "length": 0.560,
    "height": 0.115,
    "wall": 0.008,
    "floor": 0.008,
    "front_lip": 0.012,
    "rail_width": 0.018,
    "rail_height": 0.006,
    "rail_length": 0.460,
    "rail_offset": 0.118,
}

MIDDLE = {
    "width": 0.298,
    "length": 0.490,
    "height": 0.087,
    "wall": 0.007,
    "floor": 0.007,
    "front_lip": 0.010,
    "rail_width": 0.015,
    "rail_height": 0.005,
    "rail_length": 0.380,
    "rail_offset": 0.095,
    "skid_width": 0.016,
    "skid_height": 0.006,
    "skid_length": 0.420,
    "skid_offset": 0.118,
}

INNER = {
    "width": 0.256,
    "length": 0.420,
    "height": 0.062,
    "wall": 0.006,
    "floor": 0.006,
    "front_lip": 0.008,
    "rail_width": 0.012,
    "rail_height": 0.004,
    "rail_length": 0.240,
    "rail_offset": 0.067,
    "skid_width": 0.014,
    "skid_height": 0.005,
    "skid_length": 0.330,
    "skid_offset": 0.095,
}

PLATFORM = {
    "width": 0.186,
    "length": 0.240,
    "plate": 0.014,
    "fence_height": 0.025,
    "fence_thickness": 0.012,
    "foot_width": 0.012,
    "foot_height": 0.004,
    "foot_length": 0.180,
    "foot_offset": 0.067,
}

OUTER_TO_MIDDLE_Z = OUTER["height"] + OUTER["rail_height"] + MIDDLE["skid_height"]
MIDDLE_TO_INNER_Z = MIDDLE["height"] + MIDDLE["rail_height"] + INNER["skid_height"]
INNER_TO_TERMINAL_Z = INNER["height"] + INNER["rail_height"] + PLATFORM["foot_height"]


def _box(width: float, length: float, height: float, *, x: float = 0.0, y: float = 0.0, z0: float = 0.0, fillet: float = 0.0):
    solid = cq.Workplane("XY").box(width, length, height).translate((x, y, z0 + height / 2.0))
    if fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid


def _union_all(solids: list[cq.Workplane]):
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _twin_bars(offset: float, width: float, length: float, height: float, *, z0: float, y: float = 0.0, fillet: float = 0.0015):
    return _union_all(
        [
            _box(width, length, height, x=-offset, y=y, z0=z0, fillet=fillet),
            _box(width, length, height, x=offset, y=y, z0=z0, fillet=fillet),
        ]
    )


def _tray_shell(width: float, length: float, height: float, wall: float, floor: float, front_lip: float):
    shell = _union_all(
        [
            _box(width, length, floor, z0=0.0, fillet=0.0025),
            _box(wall, length, height, x=-(width - wall) / 2.0, z0=0.0, fillet=0.0020),
            _box(wall, length, height, x=(width - wall) / 2.0, z0=0.0, fillet=0.0020),
            _box(width - 2.0 * wall, wall, height, y=-(length - wall) / 2.0, z0=0.0, fillet=0.0020),
            _box(width - 2.0 * wall, wall, front_lip, y=(length - wall) / 2.0, z0=0.0, fillet=0.0015),
        ]
    )
    return shell


def _outer_tray_shape():
    return _union_all(
        [
            _tray_shell(
                OUTER["width"],
                OUTER["length"],
                OUTER["height"],
                OUTER["wall"],
                OUTER["floor"],
                OUTER["front_lip"],
            ),
            _twin_bars(
                OUTER["rail_offset"],
                OUTER["rail_width"],
                OUTER["rail_length"],
                OUTER["rail_height"],
                z0=OUTER["height"],
                y=-0.010,
            ),
        ]
    )


def _middle_tray_shape():
    return _union_all(
        [
            _tray_shell(
                MIDDLE["width"],
                MIDDLE["length"],
                MIDDLE["height"],
                MIDDLE["wall"],
                MIDDLE["floor"],
                MIDDLE["front_lip"],
            ),
            _twin_bars(
                MIDDLE["rail_offset"],
                MIDDLE["rail_width"],
                MIDDLE["rail_length"],
                MIDDLE["rail_height"],
                z0=MIDDLE["height"],
                y=-0.010,
            ),
            _twin_bars(
                MIDDLE["skid_offset"],
                MIDDLE["skid_width"],
                MIDDLE["skid_length"],
                MIDDLE["skid_height"],
                z0=-MIDDLE["skid_height"],
                y=-0.010,
            ),
        ]
    )


def _inner_tray_shape():
    return _union_all(
        [
            _tray_shell(
                INNER["width"],
                INNER["length"],
                INNER["height"],
                INNER["wall"],
                INNER["floor"],
                INNER["front_lip"],
            ),
            _twin_bars(
                INNER["rail_offset"],
                INNER["rail_width"],
                INNER["rail_length"],
                INNER["rail_height"],
                z0=INNER["height"],
                y=-0.006,
            ),
            _twin_bars(
                INNER["skid_offset"],
                INNER["skid_width"],
                INNER["skid_length"],
                INNER["skid_height"],
                z0=-INNER["skid_height"],
                y=-0.008,
            ),
        ]
    )


def _terminal_platform_shape():
    plate = _box(PLATFORM["width"], PLATFORM["length"], PLATFORM["plate"], z0=0.0, fillet=0.0025)
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(PLATFORM["width"] * 0.58, PLATFORM["length"] * 0.36)
        .cutBlind(-0.0025)
    )
    fence = _box(
        PLATFORM["width"] * 0.82,
        PLATFORM["fence_thickness"],
        PLATFORM["fence_height"],
        y=-(PLATFORM["length"] - PLATFORM["fence_thickness"]) / 2.0,
        z0=PLATFORM["plate"],
        fillet=0.0015,
    )
    feet = _twin_bars(
        PLATFORM["foot_offset"],
        PLATFORM["foot_width"],
        PLATFORM["foot_length"],
        PLATFORM["foot_height"],
        z0=-PLATFORM["foot_height"],
        y=-0.006,
        fillet=0.0012,
    )
    return _union_all([plate, fence, feet])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_drawer_shuttle")

    outer_material = model.material("outer_graphite", color=(0.23, 0.24, 0.27))
    middle_material = model.material("middle_satin", color=(0.55, 0.58, 0.60))
    inner_material = model.material("inner_light_gray", color=(0.72, 0.74, 0.76))
    platform_material = model.material("platform_black", color=(0.15, 0.15, 0.17))

    outer_tray = model.part("outer_tray")
    outer_tray.visual(
        mesh_from_cadquery(_outer_tray_shape(), "outer_tray"),
        origin=Origin(),
        material=outer_material,
        name="outer_tray_body",
    )
    outer_tray.inertial = Inertial.from_geometry(
        Box((OUTER["width"], OUTER["length"], OUTER["height"] + OUTER["rail_height"])),
        mass=1.40,
        origin=Origin(xyz=(0.0, 0.0, (OUTER["height"] + OUTER["rail_height"]) / 2.0)),
    )

    middle_tray = model.part("middle_tray")
    middle_tray.visual(
        mesh_from_cadquery(_middle_tray_shape(), "middle_tray"),
        origin=Origin(),
        material=middle_material,
        name="middle_tray_body",
    )
    middle_tray.inertial = Inertial.from_geometry(
        Box((MIDDLE["width"], MIDDLE["length"], MIDDLE["height"] + MIDDLE["rail_height"] + MIDDLE["skid_height"])),
        mass=0.95,
        origin=Origin(
            xyz=(0.0, 0.0, (MIDDLE["height"] + MIDDLE["rail_height"] - MIDDLE["skid_height"]) / 2.0)
        ),
    )

    inner_tray = model.part("inner_tray")
    inner_tray.visual(
        mesh_from_cadquery(_inner_tray_shape(), "inner_tray"),
        origin=Origin(),
        material=inner_material,
        name="inner_tray_body",
    )
    inner_tray.inertial = Inertial.from_geometry(
        Box((INNER["width"], INNER["length"], INNER["height"] + INNER["rail_height"] + INNER["skid_height"])),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, (INNER["height"] + INNER["rail_height"] - INNER["skid_height"]) / 2.0)),
    )

    terminal_platform = model.part("terminal_platform")
    terminal_platform.visual(
        mesh_from_cadquery(_terminal_platform_shape(), "terminal_platform"),
        origin=Origin(),
        material=platform_material,
        name="terminal_platform_body",
    )
    terminal_platform.inertial = Inertial.from_geometry(
        Box((PLATFORM["width"], PLATFORM["length"], PLATFORM["plate"] + PLATFORM["fence_height"] + PLATFORM["foot_height"])),
        mass=0.24,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (PLATFORM["plate"] + PLATFORM["fence_height"] - PLATFORM["foot_height"]) / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_tray,
        child=middle_tray,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=EXTENSION_AXIS,
        motion_limits=MotionLimits(effort=80.0, velocity=0.60, lower=0.0, upper=0.170),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_tray,
        child=inner_tray,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TO_INNER_Z)),
        axis=EXTENSION_AXIS,
        motion_limits=MotionLimits(effort=60.0, velocity=0.60, lower=0.0, upper=0.145),
    )
    model.articulation(
        "inner_to_terminal",
        ArticulationType.PRISMATIC,
        parent=inner_tray,
        child=terminal_platform,
        origin=Origin(xyz=(0.0, 0.0, INNER_TO_TERMINAL_Z)),
        axis=EXTENSION_AXIS,
        motion_limits=MotionLimits(effort=30.0, velocity=0.55, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tray = object_model.get_part("outer_tray")
    middle_tray = object_model.get_part("middle_tray")
    inner_tray = object_model.get_part("inner_tray")
    terminal_platform = object_model.get_part("terminal_platform")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_terminal = object_model.get_articulation("inner_to_terminal")

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

    slide_joints = (outer_to_middle, middle_to_inner, inner_to_terminal)
    ctx.check(
        "all_stages_slide_along_shared_y_axis",
        all(joint.articulation_type == ArticulationType.PRISMATIC and joint.axis == EXTENSION_AXIS for joint in slide_joints),
        details="Expected three serial prismatic joints that all translate along +Y.",
    )

    ctx.expect_contact(middle_tray, outer_tray, name="middle_tray_supported_by_outer_tray")
    ctx.expect_contact(inner_tray, middle_tray, name="inner_tray_supported_by_middle_tray")
    ctx.expect_contact(terminal_platform, inner_tray, name="terminal_platform_supported_by_inner_tray")

    ctx.expect_within(middle_tray, outer_tray, axes="x", margin=0.0, name="middle_tray_steps_inward_from_outer_tray")
    ctx.expect_within(inner_tray, middle_tray, axes="x", margin=0.0, name="inner_tray_steps_inward_from_middle_tray")
    ctx.expect_within(terminal_platform, inner_tray, axes="x", margin=0.0, name="terminal_platform_steps_inward_from_inner_tray")
    ctx.expect_origin_gap(
        middle_tray,
        outer_tray,
        axis="z",
        min_gap=OUTER_TO_MIDDLE_Z - 0.001,
        max_gap=OUTER_TO_MIDDLE_Z + 0.001,
        name="middle_stage_sits_above_outer_stage",
    )
    ctx.expect_origin_gap(
        inner_tray,
        middle_tray,
        axis="z",
        min_gap=MIDDLE_TO_INNER_Z - 0.001,
        max_gap=MIDDLE_TO_INNER_Z + 0.001,
        name="inner_stage_sits_above_middle_stage",
    )
    ctx.expect_origin_gap(
        terminal_platform,
        inner_tray,
        axis="z",
        min_gap=INNER_TO_TERMINAL_Z - 0.001,
        max_gap=INNER_TO_TERMINAL_Z + 0.001,
        name="terminal_platform_sits_above_inner_stage",
    )

    with ctx.pose(
        {
            outer_to_middle: 0.120,
            middle_to_inner: 0.090,
            inner_to_terminal: 0.060,
        }
    ):
        ctx.expect_origin_gap(
            middle_tray,
            outer_tray,
            axis="y",
            min_gap=0.119,
            max_gap=0.121,
            name="middle_stage_translates_forward",
        )
        ctx.expect_origin_gap(
            inner_tray,
            outer_tray,
            axis="y",
            min_gap=0.209,
            max_gap=0.211,
            name="inner_stage_accumulates_serial_travel",
        )
        ctx.expect_origin_gap(
            terminal_platform,
            outer_tray,
            axis="y",
            min_gap=0.269,
            max_gap=0.271,
            name="terminal_platform_accumulates_full_serial_travel",
        )
        ctx.expect_contact(middle_tray, outer_tray, name="middle_stage_remains_mounted_while_extended")
        ctx.expect_contact(inner_tray, middle_tray, name="inner_stage_remains_mounted_while_extended")
        ctx.expect_contact(terminal_platform, inner_tray, name="terminal_platform_remains_mounted_while_extended")
        ctx.expect_overlap(middle_tray, outer_tray, axes="y", min_overlap=0.320, name="outer_middle_have_slide_overlap")
        ctx.expect_overlap(inner_tray, middle_tray, axes="y", min_overlap=0.260, name="middle_inner_have_slide_overlap")
        ctx.expect_overlap(terminal_platform, inner_tray, axes="y", min_overlap=0.180, name="inner_platform_have_slide_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
