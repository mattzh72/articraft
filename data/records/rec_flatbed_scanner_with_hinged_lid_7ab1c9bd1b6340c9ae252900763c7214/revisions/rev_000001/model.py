from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_plate(width: float, depth: float, height: float, radius: float, name: str):
    """Thin rounded-rectangle solid used for molded scanner panels."""
    profile = rounded_rect_profile(width, depth, radius, corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry(profile, height, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_photo_scanner")

    warm_grey = model.material("warm_grey_plastic", rgba=(0.72, 0.72, 0.69, 1.0))
    dark_grey = model.material("dark_grey_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    glass = model.material("smoked_scanner_glass", rgba=(0.20, 0.36, 0.44, 0.42))
    white_pad = model.material("matte_white_pad", rgba=(0.96, 0.96, 0.92, 1.0))
    steel = model.material("brushed_hinge_steel", rgba=(0.70, 0.70, 0.68, 1.0))
    button_grey = model.material("button_grey", rgba=(0.25, 0.27, 0.29, 1.0))
    button_blue = model.material("scan_button_blue", rgba=(0.08, 0.34, 0.66, 1.0))
    button_green = model.material("power_button_green", rgba=(0.10, 0.48, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_plate(0.680, 0.460, 0.045, 0.035, "scanner_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=warm_grey,
        name="body_shell",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.540, 0.310),
                (0.620, 0.375),
                0.010,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.014,
                outer_corner_radius=0.026,
            ),
            "platen_bezel",
        ),
        origin=Origin(xyz=(0.0, 0.040, 0.048)),
        material=dark_grey,
        name="platen_frame",
    )
    body.visual(
        Box((0.532, 0.302, 0.004)),
        origin=Origin(xyz=(0.0, 0.040, 0.047)),
        material=glass,
        name="platen_glass",
    )
    body.visual(
        Box((0.390, 0.055, 0.006)),
        origin=Origin(xyz=(0.110, -0.198, 0.047)),
        material=dark_grey,
        name="front_control_panel",
    )
    body.visual(
        Box((0.050, 0.016, 0.003)),
        origin=Origin(xyz=(-0.035, -0.198, 0.0495)),
        material=glass,
        name="status_window",
    )

    for x in (-0.275, 0.275):
        for y in (-0.185, 0.185):
            body.visual(
                Box((0.060, 0.030, 0.005)),
                origin=Origin(xyz=(x, y, -0.0015)),
                material=black,
                name=f"rubber_foot_{x}_{y}",
            )

    hinge_axis_y = 0.238
    hinge_axis_z = 0.063
    hinge_radius = 0.006
    body.visual(
        Box((0.640, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.231, 0.048)),
        material=steel,
        name="base_hinge_leaf",
    )

    knuckle_length = 0.062
    knuckle_pitch = 0.062
    first_center = -0.279
    for index in range(10):
        center_x = first_center + index * knuckle_pitch
        if index % 2 == 0:
            body.visual(
                Box((knuckle_length, 0.007, 0.006)),
                origin=Origin(xyz=(center_x, 0.235, 0.055)),
                material=steel,
                name=f"base_hinge_tab_{index}",
            )
            body.visual(
                Cylinder(radius=hinge_radius, length=knuckle_length),
                origin=Origin(
                    xyz=(center_x, hinge_axis_y, hinge_axis_z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=steel,
                name=f"base_knuckle_{index}",
            )

    lid = model.part("lid")
    lid.visual(
        _rounded_plate(0.640, 0.400, 0.045, 0.030, "scanner_lid_shell"),
        origin=Origin(xyz=(0.0, -0.217, 0.016)),
        material=warm_grey,
        name="lid_shell",
    )
    lid.visual(
        Box((0.548, 0.318, 0.004)),
        origin=Origin(xyz=(0.0, -0.232, -0.007)),
        material=white_pad,
        name="document_pad",
    )
    lid.visual(
        Box((0.470, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, -0.418, 0.006)),
        material=dark_grey,
        name="front_grip",
    )
    lid.visual(
        Box((0.620, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.014, 0.006)),
        material=steel,
        name="lid_hinge_leaf",
    )
    for index in range(10):
        center_x = first_center + index * knuckle_pitch
        if index % 2 == 1:
            lid.visual(
                Box((knuckle_length, 0.012, 0.006)),
                origin=Origin(xyz=(center_x, -0.004, 0.003)),
                material=steel,
                name=f"lid_hinge_tab_{index}",
            )
            lid.visual(
                Cylinder(radius=hinge_radius, length=knuckle_length),
                origin=Origin(
                    xyz=(center_x, 0.0, 0.0),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=steel,
                name=f"lid_knuckle_{index}",
            )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    button_specs = (
        ("power_button", -0.020, button_green, 0.010),
        ("scan_button", 0.040, button_blue, 0.011),
        ("copy_button", 0.100, button_grey, 0.010),
    )
    for name, x_offset, material, radius in button_specs:
        button = model.part(name)
        button.visual(
            Cylinder(radius=radius, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
            material=material,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.165 + x_offset, -0.198, 0.050)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.003),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="document_pad",
            negative_elem="platen_glass",
            min_gap=0.001,
            max_gap=0.008,
            name="closed lid pad clears platen glass",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="document_pad",
            elem_b="platen_glass",
            min_overlap=0.250,
            name="closed lid covers the glass platen",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="x",
            elem_a="base_hinge_leaf",
            elem_b="lid_hinge_leaf",
            min_overlap=0.580,
            name="piano hinge runs nearly full width",
        )
        closed_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    with ctx.pose({hinge: 1.20}):
        open_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    closed_top = closed_shell_aabb[1][2] if closed_shell_aabb else None
    open_top = open_shell_aabb[1][2] if open_shell_aabb else None
    ctx.check(
        "lid opens upward on rear hinge",
        closed_top is not None and open_top is not None and open_top > closed_top + 0.25,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    return ctx.report()


object_model = build_object_model()
