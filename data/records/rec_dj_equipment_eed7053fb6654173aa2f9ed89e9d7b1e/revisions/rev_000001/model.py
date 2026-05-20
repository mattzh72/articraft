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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_modular_synth_rack")

    rack_black = model.material("rack_black", rgba=(0.11, 0.12, 0.13, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    anodized_silver = model.material("anodized_silver", rgba=(0.76, 0.78, 0.80, 1.0))
    charcoal_panel = model.material("charcoal_panel", rgba=(0.22, 0.23, 0.25, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.70, 0.72, 0.74, 1.0))
    jack_black = model.material("jack_black", rgba=(0.10, 0.10, 0.11, 1.0))
    pcb_dark = model.material("pcb_dark", rgba=(0.14, 0.18, 0.15, 1.0))
    display_glass = model.material("display_glass", rgba=(0.14, 0.28, 0.34, 0.55))
    amber_led = model.material("amber_led", rgba=(0.88, 0.54, 0.16, 0.92))
    red_led = model.material("red_led", rgba=(0.82, 0.20, 0.18, 0.92))

    outer_w = 0.28
    outer_h = 0.56
    rail_w = 0.02
    opening_h = 0.16
    inner_w = outer_w - 2.0 * rail_w
    front_depth = 0.008
    back_depth = 0.008
    frame_depth = 0.09
    frame_front_y = 0.002
    frame_back_y = -0.082
    side_run_y = -0.042
    side_run_len = 0.084

    panel_w = 0.232
    panel_h = 0.154
    panel_t = 0.004
    panel_origin_x = -inner_w * 0.5 + 0.004
    panel_origin_y = 0.013
    slot_centers_z = [
        rail_w + opening_h * 0.5 + index * (opening_h + rail_w)
        for index in range(3)
    ]

    rack_frame = model.part("rack_frame")
    rack_frame.inertial = Inertial.from_geometry(
        Box((outer_w, frame_depth, outer_h)),
        mass=7.5,
        origin=Origin(xyz=(0.0, -0.039, outer_h * 0.5)),
    )

    front_rail_x = inner_w + 0.004
    for x_sign in (-1.0, 1.0):
        rack_frame.visual(
            Box((rail_w, front_depth, outer_h)),
            origin=Origin(
                xyz=(x_sign * (outer_w * 0.5 - rail_w * 0.5), frame_front_y, outer_h * 0.5)
            ),
            material=rack_black,
        )
        rack_frame.visual(
            Box((rail_w, back_depth, outer_h)),
            origin=Origin(
                xyz=(x_sign * (outer_w * 0.5 - rail_w * 0.5), frame_back_y, outer_h * 0.5)
            ),
            material=rack_black,
        )

    horizontal_levels = [
        rail_w * 0.5,
        rail_w + opening_h + rail_w * 0.5,
        rail_w + opening_h + rail_w + opening_h + rail_w * 0.5,
        outer_h - rail_w * 0.5,
    ]
    for z_center in horizontal_levels:
        rack_frame.visual(
            Box((front_rail_x, front_depth, rail_w)),
            origin=Origin(xyz=(0.0, frame_front_y, z_center)),
            material=rack_black,
        )
        rack_frame.visual(
            Box((front_rail_x, back_depth, rail_w)),
            origin=Origin(xyz=(0.0, frame_back_y, z_center)),
            material=rack_black,
        )
        for x_sign in (-1.0, 1.0):
            rack_frame.visual(
                Box((rail_w, side_run_len, rail_w)),
                origin=Origin(
                    xyz=(x_sign * (outer_w * 0.5 - rail_w * 0.5), side_run_y, z_center)
                ),
                material=rack_black,
            )

    for x_sign in (-1.0, 1.0):
        rack_frame.visual(
            Box((0.012, front_depth + 0.010, outer_h - 0.04)),
            origin=Origin(
                xyz=(x_sign * (inner_w * 0.5 + 0.010), 0.001, outer_h * 0.5)
            ),
            material=steel_gray,
        )

    for slot_index, slot_center_z in enumerate(slot_centers_z, start=1):
        rack_frame.visual(
            Box((0.010, 0.015, opening_h - 0.010)),
            origin=Origin(xyz=(panel_origin_x - 0.005, 0.0055, slot_center_z)),
            material=steel_gray,
            name=f"left_hinge_jamb_{slot_index}",
        )

    def add_knob(
        part,
        *,
        x: float,
        z: float,
        radius: float,
        body_length: float = 0.016,
        shaft_radius: float = 0.0038,
    ) -> None:
        part.visual(
            Cylinder(radius=shaft_radius, length=0.010),
            origin=Origin(xyz=(x, 0.003, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel_gray,
        )
        part.visual(
            Cylinder(radius=radius, length=body_length),
            origin=Origin(xyz=(x, 0.012, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_black,
        )
        part.visual(
            Cylinder(radius=radius * 0.28, length=body_length * 0.94),
            origin=Origin(xyz=(x, 0.0125, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_cap,
        )

    def add_jack(part, *, x: float, z: float) -> None:
        part.visual(
            Cylinder(radius=0.0066, length=0.006),
            origin=Origin(xyz=(x, 0.001, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel_gray,
        )
        part.visual(
            Cylinder(radius=0.0046, length=0.014),
            origin=Origin(xyz=(x, 0.007, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=jack_black,
        )

    def add_led(part, *, x: float, z: float, material) -> None:
        part.visual(
            Cylinder(radius=0.003, length=0.008),
            origin=Origin(xyz=(x, 0.004, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=material,
        )

    def add_button(part, *, x: float, z: float) -> None:
        part.visual(
            Box((0.012, 0.008, 0.012)),
            origin=Origin(xyz=(x, 0.004, z)),
            material=charcoal_panel,
        )

    def add_toggle(part, *, x: float, z: float) -> None:
        part.visual(
            Cylinder(radius=0.0036, length=0.009),
            origin=Origin(xyz=(x, 0.0035, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel_gray,
        )
        part.visual(
            Box((0.003, 0.018, 0.012)),
            origin=Origin(xyz=(x, 0.013, z)),
            material=knob_black,
        )

    def add_panel_shell(part, *, face_material, rear_width: float) -> None:
        part.visual(
            Box((panel_w, panel_t, panel_h)),
            origin=Origin(xyz=(panel_w * 0.5, 0.0, 0.0)),
            material=face_material,
            name="panel_face",
        )
        part.visual(
            Box((0.014, 0.012, panel_h)),
            origin=Origin(xyz=(0.007, 0.006, 0.0)),
            material=steel_gray,
            name="hinge_strip",
        )
        part.visual(
            Box((rear_width, 0.054, panel_h * 0.60)),
            origin=Origin(xyz=(0.132, -0.024, 0.0)),
            material=pcb_dark,
        )
        part.visual(
            Box((0.010, 0.016, panel_h * 0.88)),
            origin=Origin(xyz=(panel_w - 0.004, 0.006, 0.0)),
            material=steel_gray,
        )
        for screw_x in (0.020, panel_w - 0.020):
            for screw_z in (-panel_h * 0.5 + 0.016, panel_h * 0.5 - 0.016):
                part.visual(
                    Sphere(radius=0.0022),
                    origin=Origin(xyz=(screw_x, 0.004, screw_z)),
                    material=steel_gray,
                )

    module_1 = model.part("module_panel_1")
    add_panel_shell(module_1, face_material=anodized_silver, rear_width=0.152)
    module_1.inertial = Inertial.from_geometry(
        Box((panel_w, 0.058, panel_h)),
        mass=0.68,
        origin=Origin(xyz=(panel_w * 0.5, -0.020, 0.0)),
    )
    module_1.visual(
        Box((0.170, 0.002, 0.018)),
        origin=Origin(xyz=(0.118, 0.003, 0.058)),
        material=charcoal_panel,
    )
    add_led(module_1, x=0.202, z=0.058, material=amber_led)
    for x in (0.066, 0.162):
        add_knob(module_1, x=x, z=0.028, radius=0.014)
    for x in (0.052, 0.112, 0.172):
        add_knob(module_1, x=x, z=-0.024, radius=0.010)
    for x in (0.044, 0.090, 0.136, 0.182):
        add_jack(module_1, x=x, z=-0.060)

    module_2 = model.part("module_panel_2")
    add_panel_shell(module_2, face_material=charcoal_panel, rear_width=0.158)
    module_2.inertial = Inertial.from_geometry(
        Box((panel_w, 0.058, panel_h)),
        mass=0.72,
        origin=Origin(xyz=(panel_w * 0.5, -0.020, 0.0)),
    )
    add_knob(module_2, x=0.088, z=0.032, radius=0.017, body_length=0.018)
    for x in (0.052, 0.112, 0.172):
        add_knob(module_2, x=x, z=-0.014, radius=0.010)
    module_2.visual(
        Box((0.008, 0.006, 0.070)),
        origin=Origin(xyz=(0.188, 0.003, 0.018)),
        material=steel_gray,
    )
    module_2.visual(
        Box((0.018, 0.010, 0.012)),
        origin=Origin(xyz=(0.188, 0.010, 0.050)),
        material=knob_black,
    )
    for x in (0.050, 0.082):
        add_button(module_2, x=x, z=0.060)
    add_toggle(module_2, x=0.128, z=0.056)
    add_led(module_2, x=0.154, z=0.056, material=red_led)
    for x in (0.046, 0.094, 0.142, 0.190):
        add_jack(module_2, x=x, z=-0.060)
    for x in (0.070, 0.166):
        add_jack(module_2, x=x, z=-0.036)

    module_3 = model.part("module_panel_3")
    add_panel_shell(module_3, face_material=anodized_silver, rear_width=0.148)
    module_3.inertial = Inertial.from_geometry(
        Box((panel_w, 0.058, panel_h)),
        mass=0.65,
        origin=Origin(xyz=(panel_w * 0.5, -0.020, 0.0)),
    )
    module_3.visual(
        Box((0.080, 0.010, 0.028)),
        origin=Origin(xyz=(0.090, 0.007, 0.032)),
        material=display_glass,
    )
    for x in (0.170, 0.204):
        add_knob(module_3, x=x, z=0.036, radius=0.009, body_length=0.014)
    for x in (0.054, 0.078, 0.102, 0.126):
        add_button(module_3, x=x, z=-0.010)
    add_led(module_3, x=0.154, z=-0.006, material=amber_led)
    for x in (0.048, 0.094, 0.140, 0.186):
        add_jack(module_3, x=x, z=-0.058)

    for index, (panel, slot_center_z) in enumerate(
        zip((module_1, module_2, module_3), slot_centers_z),
        start=1,
    ):
        model.articulation(
            f"rack_to_module_panel_{index}",
            ArticulationType.REVOLUTE,
            parent=rack_frame,
            child=panel,
            origin=Origin(xyz=(panel_origin_x, panel_origin_y, slot_center_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.8,
                lower=0.0,
                upper=1.95,
            ),
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

    rack_frame = object_model.get_part("rack_frame")
    panels = [
        object_model.get_part("module_panel_1"),
        object_model.get_part("module_panel_2"),
        object_model.get_part("module_panel_3"),
    ]
    hinges = [
        object_model.get_articulation("rack_to_module_panel_1"),
        object_model.get_articulation("rack_to_module_panel_2"),
        object_model.get_articulation("rack_to_module_panel_3"),
    ]

    for index, (panel, hinge) in enumerate(zip(panels, hinges), start=1):
        limits = hinge.motion_limits
        ctx.check(
            f"module panel {index} hinge axis and limits",
            hinge.axis == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 1.8,
            details=f"axis={hinge.axis}, limits={limits}",
        )

        with ctx.pose({hinge: 0.0}):
            ctx.expect_overlap(
                panel,
                rack_frame,
                axes="xz",
                elem_a="panel_face",
                min_overlap=0.12,
                name=f"module panel {index} sits within the rack face",
            )
            ctx.expect_origin_gap(
                panel,
                rack_frame,
                axis="y",
                min_gap=0.008,
                max_gap=0.014,
                name=f"module panel {index} hinge line sits slightly proud of frame",
            )
            closed_aabb = ctx.part_element_world_aabb(panel, elem="panel_face")

        with ctx.pose({hinge: 1.05}):
            open_aabb = ctx.part_element_world_aabb(panel, elem="panel_face")

        if closed_aabb is not None and open_aabb is not None:
            closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
            open_center_y = (open_aabb[0][1] + open_aabb[1][1]) * 0.5
            ctx.check(
                f"module panel {index} swings forward for cable access",
                open_center_y > closed_center_y + 0.075,
                details=f"closed_center_y={closed_center_y}, open_center_y={open_center_y}",
            )
        else:
            ctx.fail(
                f"module panel {index} panel face aabb available",
                details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
