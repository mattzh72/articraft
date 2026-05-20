from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _radial_origin(
    radius: float,
    z: float,
    yaw: float,
) -> Origin:
    return Origin(
        xyz=(radius * math.cos(yaw), radius * math.sin(yaw), z),
        rpy=(0.0, 0.0, yaw),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="frameless_glass_revolving_door")

    glass = model.material("glass", rgba=(0.73, 0.88, 0.95, 0.18))
    fixed_glass = model.material("fixed_glass", rgba=(0.76, 0.92, 0.98, 0.15))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.45, 0.47, 0.50, 1.0))
    stone = model.material("stone", rgba=(0.78, 0.77, 0.74, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))

    door_radius = 1.12
    drum_radius = 1.16
    canopy_radius = 1.48
    floor_top = 0.04
    canopy_bottom = 2.64
    canopy_thickness = 0.08
    fixed_glass_height = canopy_bottom - floor_top
    wing_glass_height = 2.52
    wing_glass_thickness = 0.012
    post_radius = 0.045

    storefront = model.part("storefront")
    storefront.visual(
        Cylinder(radius=1.55, length=floor_top),
        origin=Origin(xyz=(0.0, 0.0, floor_top * 0.5)),
        material=stone,
        name="threshold_disc",
    )
    storefront.visual(
        Cylinder(radius=canopy_radius, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, canopy_bottom + canopy_thickness * 0.5)),
        material=charcoal,
        name="canopy_disc",
    )
    storefront.visual(
        Cylinder(radius=0.22, length=canopy_thickness * 0.65),
        origin=Origin(xyz=(0.0, 0.0, canopy_bottom + canopy_thickness * 0.325)),
        material=satin_steel,
        name="ceiling_bearing_housing",
    )

    fixed_panel_width = 0.94
    fixed_panel_angles = (
        math.radians(20.0),
        math.radians(-20.0),
        math.radians(160.0),
        math.radians(200.0),
    )
    for index, angle in enumerate(fixed_panel_angles, start=1):
        storefront.visual(
            Box((fixed_panel_width, wing_glass_thickness, fixed_glass_height)),
            origin=Origin(
                xyz=(
                    drum_radius * math.cos(angle),
                    drum_radius * math.sin(angle),
                    floor_top + fixed_glass_height * 0.5,
                ),
                rpy=(0.0, 0.0, angle + math.pi * 0.5),
            ),
            material=fixed_glass,
            name=f"fixed_drum_glass_{index}",
        )

    storefront.inertial = Inertial.from_geometry(
        Box((3.10, 3.10, 2.72)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.36)),
    )

    wing_assembly = model.part("wing_assembly")
    wing_assembly.visual(
        Cylinder(radius=post_radius, length=2.60),
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        material=brushed_steel,
        name="central_post",
    )
    wing_assembly.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=satin_steel,
        name="bottom_pivot_collar",
    )
    wing_assembly.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.56)),
        material=satin_steel,
        name="top_pivot_collar",
    )
    wing_assembly.visual(
        Cylinder(radius=0.11, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=brushed_steel,
        name="bottom_hub_plate",
    )
    wing_assembly.visual(
        Cylinder(radius=0.11, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 2.515)),
        material=brushed_steel,
        name="top_hub_plate",
    )

    wing_panel_length = door_radius - post_radius - 0.01
    wing_panel_center_radius = post_radius + wing_panel_length * 0.5
    patch_length = 0.20
    patch_width = 0.06
    patch_height = 0.045
    patch_center_radius = 0.14
    wing_angles = (
        math.pi * 0.5,
        math.pi * 0.5 + 2.0 * math.pi / 3.0,
        math.pi * 0.5 + 4.0 * math.pi / 3.0,
    )
    for index, angle in enumerate(wing_angles, start=1):
        wing_assembly.visual(
            Box((wing_panel_length, wing_glass_thickness, wing_glass_height)),
            origin=_radial_origin(
                radius=wing_panel_center_radius,
                z=wing_glass_height * 0.5 + 0.008,
                yaw=angle,
            ),
            material=glass,
            name=f"wing_{index}_glass",
        )
        wing_assembly.visual(
            Box((patch_length, patch_width, patch_height)),
            origin=_radial_origin(
                radius=patch_center_radius,
                z=0.055,
                yaw=angle,
            ),
            material=satin_steel,
            name=f"wing_{index}_bottom_patch",
        )
        wing_assembly.visual(
            Box((patch_length, patch_width, patch_height)),
            origin=_radial_origin(
                radius=patch_center_radius,
                z=2.505,
                yaw=angle,
            ),
            material=satin_steel,
            name=f"wing_{index}_top_patch",
        )

    wing_assembly.inertial = Inertial.from_geometry(
        Box((2.30, 2.30, 2.60)),
        mass=70.0,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
    )

    model.articulation(
        "door_rotation",
        ArticulationType.CONTINUOUS,
        parent=storefront,
        child=wing_assembly,
        origin=Origin(xyz=(0.0, 0.0, floor_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4),
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
    storefront = object_model.get_part("storefront")
    wing_assembly = object_model.get_part("wing_assembly")
    door_rotation = object_model.get_articulation("door_rotation")

    ctx.check(
        "revolving door uses continuous vertical rotation",
        door_rotation.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in door_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"type={door_rotation.joint_type}, axis={door_rotation.axis}",
    )
    ctx.expect_contact(
        wing_assembly,
        storefront,
        elem_a="top_pivot_collar",
        elem_b="canopy_disc",
        name="top pivot collar seats against the canopy bearing",
    )
    ctx.expect_contact(
        wing_assembly,
        storefront,
        elem_a="bottom_pivot_collar",
        elem_b="threshold_disc",
        name="bottom pivot collar lands on the threshold pivot",
    )
    ctx.expect_gap(
        wing_assembly,
        storefront,
        axis="z",
        positive_elem="wing_1_glass",
        negative_elem="threshold_disc",
        min_gap=0.007,
        max_gap=0.0095,
        name="wing glass clears the stone threshold by a slim sweep gap",
    )
    ctx.expect_gap(
        storefront,
        wing_assembly,
        axis="z",
        positive_elem="canopy_disc",
        negative_elem="wing_1_glass",
        min_gap=0.07,
        max_gap=0.08,
        name="wing glass stops below the canopy patch hardware zone",
    )
    for index in (1, 2, 3):
        ctx.expect_within(
            wing_assembly,
            storefront,
            axes="xy",
            inner_elem=f"wing_{index}_glass",
            outer_elem="canopy_disc",
            margin=0.0,
            name=f"wing {index} stays within the revolving drum footprint",
        )

    def _panel_center(name: str):
        return _aabb_center(ctx.part_element_world_aabb(wing_assembly, elem=name))

    rest_center = _panel_center("wing_1_glass")
    with ctx.pose({door_rotation: 2.0 * math.pi / 3.0}):
        turned_center = _panel_center("wing_1_glass")

    rest_angle = math.atan2(rest_center[1], rest_center[0]) if rest_center else None
    turned_angle = math.atan2(turned_center[1], turned_center[0]) if turned_center else None
    if rest_angle is not None and turned_angle is not None:
        delta = turned_angle - rest_angle
        while delta <= -math.pi:
            delta += 2.0 * math.pi
        while delta > math.pi:
            delta -= 2.0 * math.pi
    else:
        delta = None

    rest_radius = (
        math.hypot(rest_center[0], rest_center[1])
        if rest_center is not None
        else None
    )
    turned_radius = (
        math.hypot(turned_center[0], turned_center[1])
        if turned_center is not None
        else None
    )
    ctx.check(
        "one wing advances by roughly 120 degrees with preserved sweep radius",
        rest_center is not None
        and turned_center is not None
        and delta is not None
        and abs(delta - 2.0 * math.pi / 3.0) < 0.08
        and rest_radius is not None
        and turned_radius is not None
        and abs(rest_radius - turned_radius) < 0.01,
        details=(
            f"rest_center={rest_center}, turned_center={turned_center}, "
            f"delta={delta}, rest_radius={rest_radius}, turned_radius={turned_radius}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
