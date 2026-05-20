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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


CORE_SIZE = 0.060
BEZEL_THICKNESS = 0.002
CORE_HALF = CORE_SIZE * 0.5
OVERALL_SIZE = CORE_SIZE + 2.0 * BEZEL_THICKNESS
BEZEL_OUTER = 0.056
BEZEL_OPENING = 0.049
PANEL_SIZE = 0.046
PANEL_THICKNESS = BEZEL_THICKNESS
AXLE_RADIUS = 0.0028
AXLE_HOLE_RADIUS = 0.0036
INDICATOR_THICKNESS = 0.0005

FACE_SPECS = (
    {
        "label": "top",
        "face": "+z",
        "axis": (0.0, 0.0, 1.0),
        "normal_axis": "z",
        "plane_axes": "xy",
        "positive_dial": True,
        "joint_xyz": (0.0, 0.0, CORE_HALF),
    },
    {
        "label": "bottom",
        "face": "-z",
        "axis": (0.0, 0.0, -1.0),
        "normal_axis": "z",
        "plane_axes": "xy",
        "positive_dial": False,
        "joint_xyz": (0.0, 0.0, -CORE_HALF),
    },
    {
        "label": "right",
        "face": "+x",
        "axis": (1.0, 0.0, 0.0),
        "normal_axis": "x",
        "plane_axes": "yz",
        "positive_dial": True,
        "joint_xyz": (CORE_HALF, 0.0, 0.0),
    },
    {
        "label": "left",
        "face": "-x",
        "axis": (-1.0, 0.0, 0.0),
        "normal_axis": "x",
        "plane_axes": "yz",
        "positive_dial": False,
        "joint_xyz": (-CORE_HALF, 0.0, 0.0),
    },
    {
        "label": "front",
        "face": "+y",
        "axis": (0.0, 1.0, 0.0),
        "normal_axis": "y",
        "plane_axes": "xz",
        "positive_dial": True,
        "joint_xyz": (0.0, CORE_HALF, 0.0),
    },
    {
        "label": "back",
        "face": "-y",
        "axis": (0.0, -1.0, 0.0),
        "normal_axis": "y",
        "plane_axes": "xz",
        "positive_dial": False,
        "joint_xyz": (0.0, -CORE_HALF, 0.0),
    },
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _face_rpy(face: str) -> tuple[float, float, float]:
    if face == "+z":
        return (0.0, 0.0, 0.0)
    if face == "-z":
        return (pi, 0.0, 0.0)
    if face == "+x":
        return (0.0, pi / 2.0, 0.0)
    if face == "-x":
        return (0.0, -pi / 2.0, 0.0)
    if face == "+y":
        return (-pi / 2.0, 0.0, 0.0)
    if face == "-y":
        return (pi / 2.0, 0.0, 0.0)
    raise ValueError(f"Unsupported face {face!r}")


def _face_offset(face: str, *, depth: float, a: float = 0.0, b: float = 0.0) -> tuple[float, float, float]:
    if face == "+z":
        return (a, b, depth)
    if face == "-z":
        return (a, b, -depth)
    if face == "+x":
        return (depth, a, b)
    if face == "-x":
        return (-depth, a, b)
    if face == "+y":
        return (a, depth, b)
    if face == "-y":
        return (a, -depth, b)
    raise ValueError(f"Unsupported face {face!r}")


def _face_origin(face: str, *, depth: float, a: float = 0.0, b: float = 0.0) -> Origin:
    return Origin(xyz=_face_offset(face, depth=depth, a=a, b=b), rpy=_face_rpy(face))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cube_fidget_toy")

    body_shell = model.material("body_shell", rgba=(0.93, 0.94, 0.96, 1.0))
    bezel_dark = model.material("bezel_dark", rgba=(0.17, 0.19, 0.22, 1.0))
    axle_dark = model.material("axle_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    indicator_light = model.material("indicator_light", rgba=(0.96, 0.96, 0.97, 1.0))

    panel_materials = {
        "top": model.material("panel_top_blue", rgba=(0.25, 0.53, 0.93, 1.0)),
        "bottom": model.material("panel_bottom_yellow", rgba=(0.95, 0.81, 0.25, 1.0)),
        "right": model.material("panel_right_red", rgba=(0.87, 0.31, 0.29, 1.0)),
        "left": model.material("panel_left_green", rgba=(0.28, 0.70, 0.49, 1.0)),
        "front": model.material("panel_front_orange", rgba=(0.92, 0.54, 0.20, 1.0)),
        "back": model.material("panel_back_purple", rgba=(0.55, 0.41, 0.83, 1.0)),
    }

    bezel_outer_profile = rounded_rect_profile(BEZEL_OUTER, BEZEL_OUTER, radius=0.008, corner_segments=10)
    bezel_hole_profile = rounded_rect_profile(BEZEL_OPENING, BEZEL_OPENING, radius=0.011, corner_segments=10)
    panel_outer_profile = rounded_rect_profile(PANEL_SIZE, PANEL_SIZE, radius=0.011, corner_segments=12)
    axle_hole_profile = superellipse_profile(
        AXLE_HOLE_RADIUS * 2.0,
        AXLE_HOLE_RADIUS * 2.0,
        exponent=2.0,
        segments=28,
    )

    bezel_mesh = _mesh(
        "cube_face_bezel",
        ExtrudeWithHolesGeometry(
            bezel_outer_profile,
            [bezel_hole_profile],
            BEZEL_THICKNESS,
            center=False,
        ),
    )
    panel_mesh = _mesh(
        "cube_face_panel",
        ExtrudeWithHolesGeometry(
            panel_outer_profile,
            [axle_hole_profile],
            PANEL_THICKNESS,
            center=False,
        ),
    )

    body = model.part("cube_body")
    body.visual(Box((CORE_SIZE, CORE_SIZE, CORE_SIZE)), material=body_shell, name="core_shell")
    body.inertial = Inertial.from_geometry(Box((OVERALL_SIZE, OVERALL_SIZE, OVERALL_SIZE)), mass=0.18)

    for spec in FACE_SPECS:
        label = spec["label"]
        face = spec["face"]
        body.visual(
            bezel_mesh,
            origin=_face_origin(face, depth=CORE_HALF),
            material=bezel_dark,
            name=f"{label}_bezel",
        )
        body.visual(
            Cylinder(radius=AXLE_RADIUS, length=BEZEL_THICKNESS),
            origin=_face_origin(face, depth=CORE_HALF + BEZEL_THICKNESS * 0.5),
            material=axle_dark,
            name=f"{label}_axle",
        )

    for spec in FACE_SPECS:
        label = spec["label"]
        face = spec["face"]
        panel = model.part(f"{label}_panel")
        panel.visual(
            panel_mesh,
            origin=_face_origin(face, depth=0.0),
            material=panel_materials[label],
            name="shell",
        )
        panel.visual(
            Box((0.004, 0.020, INDICATOR_THICKNESS)),
            origin=Origin(
                xyz=_face_offset(
                    face,
                    depth=PANEL_THICKNESS + INDICATOR_THICKNESS * 0.5,
                    a=0.0,
                    b=0.012,
                ),
                rpy=_face_rpy(face),
            ),
            material=indicator_light,
            name="indicator_bar",
        )
        panel.visual(
            Box((0.010, 0.004, INDICATOR_THICKNESS)),
            origin=Origin(
                xyz=_face_offset(
                    face,
                    depth=PANEL_THICKNESS + INDICATOR_THICKNESS * 0.5,
                    a=0.0,
                    b=0.019,
                ),
                rpy=_face_rpy(face),
            ),
            material=indicator_light,
            name="indicator_tip",
        )

        model.articulation(
            f"{label}_panel_spin",
            ArticulationType.REVOLUTE,
            parent=body,
            child=panel,
            origin=Origin(xyz=spec["joint_xyz"]),
            axis=spec["axis"],
            motion_limits=MotionLimits(effort=0.2, velocity=12.0, lower=-pi, upper=pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cube_body")
    joints = {spec["label"]: object_model.get_articulation(f"{spec['label']}_panel_spin") for spec in FACE_SPECS}
    panels = {spec["label"]: object_model.get_part(f"{spec['label']}_panel") for spec in FACE_SPECS}

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

    ctx.check("part_count", len(object_model.parts) == 7, f"expected 7 parts, found {len(object_model.parts)}")
    ctx.check(
        "articulation_count",
        len(object_model.articulations) == 6,
        f"expected 6 articulations, found {len(object_model.articulations)}",
    )

    for spec in FACE_SPECS:
        label = spec["label"]
        panel = panels[label]
        joint = joints[label]

        ctx.check(
            f"{label}_joint_axis",
            tuple(joint.axis) == tuple(spec["axis"]),
            f"expected axis {spec['axis']}, got {joint.axis}",
        )
        ctx.expect_contact(
            panel,
            body,
            elem_a="shell",
            elem_b="core_shell",
            name=f"{label}_panel_seated_on_core",
        )
        ctx.expect_origin_distance(
            panel,
            body,
            axes=spec["plane_axes"],
            max_dist=1e-6,
            name=f"{label}_panel_centered_in_face_plane",
        )
        if spec["positive_dial"]:
            ctx.expect_origin_gap(
                panel,
                body,
                axis=spec["normal_axis"],
                min_gap=CORE_HALF,
                max_gap=CORE_HALF,
                name=f"{label}_panel_on_correct_face",
            )
            ctx.expect_gap(
                panel,
                body,
                axis=spec["normal_axis"],
                positive_elem="shell",
                negative_elem="core_shell",
                max_gap=1e-6,
                max_penetration=1e-6,
                name=f"{label}_panel_flush_to_body_core",
            )
        else:
            ctx.expect_origin_gap(
                body,
                panel,
                axis=spec["normal_axis"],
                min_gap=CORE_HALF,
                max_gap=CORE_HALF,
                name=f"{label}_panel_on_correct_face",
            )
            ctx.expect_gap(
                body,
                panel,
                axis=spec["normal_axis"],
                positive_elem="core_shell",
                negative_elem="shell",
                max_gap=1e-6,
                max_penetration=1e-6,
                name=f"{label}_panel_flush_to_body_core",
            )

    with ctx.pose({joints["top"]: 1.2, joints["front"]: -0.9}):
        ctx.expect_contact(
            panels["top"],
            body,
            elem_a="shell",
            elem_b="core_shell",
            name="top_panel_stays_seated_when_spun",
        )
        ctx.expect_contact(
            panels["front"],
            body,
            elem_a="shell",
            elem_b="core_shell",
            name="front_panel_stays_seated_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
