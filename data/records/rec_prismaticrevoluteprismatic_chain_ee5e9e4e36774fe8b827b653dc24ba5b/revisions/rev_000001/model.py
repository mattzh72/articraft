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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _centered_z_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -0.5 * length))


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return _centered_z_cylinder(radius, length).rotate((0, 0, 0), (0, 1, 0), 90).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return _centered_z_cylinder(radius, length).rotate((0, 0, 0), (1, 0, 0), -90).translate(center)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return _centered_z_cylinder(radius, length).translate(center)


def _build_support_shape() -> cq.Workplane:
    rail_radius = 0.014

    support = _box((0.52, 0.30, 0.06), (0.0, -0.08, 0.03))
    support = support.union(_box((0.10, 0.12, 0.82), (-0.18, -0.08, 0.47)))
    support = support.union(_box((0.10, 0.12, 0.82), (0.18, -0.08, 0.47)))
    support = support.union(_box((0.46, 0.10, 0.08), (0.0, -0.08, 0.88)))
    support = support.union(_box((0.22, 0.02, 0.68), (0.0, -0.13, 0.53)))
    support = support.union(_box((0.08, 0.04, 0.72), (0.0, 0.00, 0.55)))
    support = support.union(_box((0.26, 0.05, 0.03), (0.0, -0.005, 0.22)))
    support = support.union(_box((0.26, 0.05, 0.03), (0.0, -0.005, 0.88)))
    support = support.union(_z_cylinder(rail_radius, 0.66, (-0.11, 0.00, 0.55)))
    support = support.union(_z_cylinder(rail_radius, 0.66, (0.11, 0.00, 0.55)))
    support = support.union(_box((0.06, 0.07, 0.26), (0.0, 0.035, 0.55)))
    return support


def _build_carriage_shape() -> cq.Workplane:
    carriage = _box((0.08, 0.012, 0.18), (0.0, 0.006, 0.0))
    carriage = carriage.union(_box((0.16, 0.045, 0.10), (0.0, 0.0345, 0.0)))
    carriage = carriage.union(_box((0.08, 0.015, 0.08), (0.0, 0.0625, 0.0)))
    carriage = carriage.union(_box((0.035, 0.018, 0.12), (-0.05, 0.032, 0.0)))
    carriage = carriage.union(_box((0.035, 0.018, 0.12), (0.05, 0.032, 0.0)))
    return carriage


def _build_frame_shape() -> cq.Workplane:
    frame = _box((0.05, 0.03, 0.09), (0.0, 0.015, 0.0))
    frame = frame.union(_box((0.018, 0.24, 0.018), (-0.016, 0.15, 0.038)))
    frame = frame.union(_box((0.018, 0.24, 0.018), (0.016, 0.15, 0.038)))
    frame = frame.union(_box((0.018, 0.24, 0.018), (-0.016, 0.15, -0.038)))
    frame = frame.union(_box((0.018, 0.24, 0.018), (0.016, 0.15, -0.038)))
    frame = frame.union(_box((0.05, 0.04, 0.10), (0.0, 0.26, 0.0)))
    frame = frame.union(_y_cylinder(0.022, 0.05, (0.0, 0.295, 0.0)))

    frame = frame.cut(_y_cylinder(0.0165, 0.07, (0.0, 0.295, 0.0)))
    return frame


def _build_ram_shape() -> cq.Workplane:
    ram = _y_cylinder(0.014, 0.18, (0.0, 0.09, 0.0))
    ram = ram.union(_y_cylinder(0.018, 0.012, (0.0, 0.006, 0.0)))
    ram = ram.union(_box((0.055, 0.04, 0.05), (0.0, 0.17, 0.0)))
    ram = ram.union(_box((0.08, 0.018, 0.07), (0.0, 0.194, 0.0)))
    return ram


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_transfer_arm")

    model.material("support_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("carriage_orange", rgba=(0.88, 0.46, 0.12, 1.0))
    model.material("frame_light", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("ram_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_build_support_shape(), "rear_support"),
        material="support_gray",
        name="support_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "carriage"),
        material="carriage_orange",
        name="carriage_shell",
    )

    hinged_frame = model.part("hinged_frame")
    hinged_frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "hinged_frame"),
        material="frame_light",
        name="frame_shell",
    )

    output_ram = model.part("output_ram")
    output_ram.visual(
        mesh_from_cadquery(_build_ram_shape(), "output_ram"),
        material="ram_steel",
        name="ram_shell",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_support,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.07, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=700.0,
            velocity=0.30,
            lower=-0.18,
            upper=0.18,
        ),
    )

    model.articulation(
        "carriage_to_frame",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=hinged_frame,
        origin=Origin(xyz=(0.0, 0.07, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.95,
        ),
    )

    model.articulation(
        "frame_to_ram",
        ArticulationType.PRISMATIC,
        parent=hinged_frame,
        child=output_ram,
        origin=Origin(xyz=(0.0, 0.32, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.20,
            lower=0.0,
            upper=0.09,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    carriage = object_model.get_part("carriage")
    hinged_frame = object_model.get_part("hinged_frame")
    output_ram = object_model.get_part("output_ram")

    carriage_slide = object_model.get_articulation("support_to_carriage")
    frame_hinge = object_model.get_articulation("carriage_to_frame")
    ram_slide = object_model.get_articulation("frame_to_ram")

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

    ctx.expect_contact(
        carriage,
        rear_support,
        contact_tol=0.002,
        name="carriage supported by rear rails",
    )
    ctx.expect_contact(
        hinged_frame,
        carriage,
        contact_tol=0.002,
        name="frame carried on carriage hinge",
    )
    ctx.expect_contact(
        output_ram,
        hinged_frame,
        contact_tol=0.002,
        name="ram guided by front sleeve",
    )

    lower_slide = carriage_slide.motion_limits.lower
    upper_slide = carriage_slide.motion_limits.upper
    if lower_slide is None or upper_slide is None:
        ctx.fail("carriage slide limits present", "support_to_carriage missing lower/upper limits")
    else:
        with ctx.pose({carriage_slide: lower_slide}):
            carriage_low = ctx.part_world_position(carriage)
        with ctx.pose({carriage_slide: upper_slide}):
            carriage_high = ctx.part_world_position(carriage)
        ctx.check(
            "positive carriage slide raises the carriage",
            carriage_low is not None
            and carriage_high is not None
            and carriage_high[2] > carriage_low[2] + 0.35,
            details=f"low={carriage_low}, high={carriage_high}",
        )

    upper_hinge = frame_hinge.motion_limits.upper
    if upper_hinge is None:
        ctx.fail("frame hinge upper limit present", "carriage_to_frame missing upper limit")
    else:
        with ctx.pose({frame_hinge: 0.0}):
            frame_level = ctx.part_world_aabb(hinged_frame)
        with ctx.pose({frame_hinge: upper_hinge}):
            frame_raised = ctx.part_world_aabb(hinged_frame)
        ctx.check(
            "positive hinge lifts the frame nose",
            frame_level is not None
            and frame_raised is not None
            and frame_raised[1][2] > frame_level[1][2] + 0.18,
            details=f"level={frame_level}, raised={frame_raised}",
        )

    upper_ram = ram_slide.motion_limits.upper
    if upper_ram is None:
        ctx.fail("ram slide upper limit present", "frame_to_ram missing upper limit")
    else:
        with ctx.pose({ram_slide: 0.0}):
            ram_retracted = ctx.part_world_aabb(output_ram)
        with ctx.pose({ram_slide: upper_ram}):
            ram_extended = ctx.part_world_aabb(output_ram)
        ctx.check(
            "positive ram slide extends the nose forward",
            ram_retracted is not None
            and ram_extended is not None
            and ram_extended[1][1] > ram_retracted[1][1] + 0.08,
            details=f"retracted={ram_retracted}, extended={ram_extended}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
