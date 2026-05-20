from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

CARD_LENGTH = 0.286
CARD_HEIGHT = 0.108
PCB_THICKNESS = 0.002
FIN_BASE_THICKNESS = 0.004
HEATSINK_TOP_Z = 0.022
SHROUD_BOTTOM_Z = HEATSINK_TOP_Z
SHROUD_TOP_Z = 0.038
BACKPLATE_THICKNESS = 0.0016
FAN_CENTERS = (-0.092, 0.0, 0.092)
FAN_RIM_OUTER = 0.039
FAN_RIM_INNER = 0.0355
SHROUD_BEZEL_OUTER = 0.045
SHROUD_BEZEL_INNER = 0.0408
FAN_Z_JOINT = 0.0


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _ring_band_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    radial_segments: int = 56,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.002,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner)


def _add_fin_stack(part, material) -> None:
    part.visual(
        Box((0.266, 0.102, FIN_BASE_THICKNESS)),
        origin=Origin(xyz=(0.004, 0.000, 0.004)),
        material=material,
        name="fin_base",
    )
    part.visual(
        Box((0.256, 0.096, 0.002)),
        origin=Origin(xyz=(0.008, 0.000, HEATSINK_TOP_Z - 0.001)),
        material=material,
        name="top_spreader",
    )
    fin_count = 15
    for index in range(fin_count):
        y = -0.044 + index * (0.088 / (fin_count - 1))
        part.visual(
            Box((0.250, 0.0018, HEATSINK_TOP_Z - 0.006)),
            origin=Origin(xyz=(0.008, y, 0.014)),
            material=material,
            name=f"fin_{index}",
        )
    part.visual(
        Box((0.020, 0.100, 0.016)),
        origin=Origin(xyz=(-0.118, 0.000, 0.014)),
        material=material,
        name="left_cold_plate",
    )
    part.visual(
        Box((0.020, 0.100, 0.016)),
        origin=Origin(xyz=(0.126, 0.000, 0.014)),
        material=material,
        name="right_cold_plate",
    )


def _add_core_details(part, pcb_material, metal_material, gold_material) -> None:
    part.visual(
        Box((CARD_LENGTH, CARD_HEIGHT, PCB_THICKNESS)),
        origin=Origin(xyz=(0.004, 0.000, PCB_THICKNESS * 0.5)),
        material=pcb_material,
        name="pcb",
    )
    part.visual(
        Box((0.092, 0.008, 0.0012)),
        origin=Origin(xyz=(-0.018, -0.050, 0.0014)),
        material=gold_material,
        name="pcie_fingers",
    )
    part.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.084, 0.060, 0.007)),
        material=metal_material,
        name="power_socket_left",
    )
    part.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.108, 0.060, 0.007)),
        material=metal_material,
        name="power_socket_right",
    )
    part.visual(
        Box((0.118, 0.006, 0.008)),
        origin=Origin(xyz=(0.070, 0.052, 0.006)),
        material=metal_material,
        name="top_edge_bar",
    )
    part.visual(
        Box((0.074, 0.010, 0.008)),
        origin=Origin(xyz=(0.102, -0.044, 0.006)),
        material=metal_material,
        name="rear_stiffener",
    )


def _add_shroud_structure(part, body_material, accent_material, bezel_mesh) -> None:
    frame_center_z = 0.030
    frame_height = SHROUD_TOP_Z - SHROUD_BOTTOM_Z

    part.visual(
        Box((0.282, 0.012, frame_height)),
        origin=Origin(xyz=(0.000, 0.051, frame_center_z)),
        material=body_material,
        name="top_rail",
    )
    part.visual(
        Box((0.282, 0.012, frame_height)),
        origin=Origin(xyz=(0.000, -0.051, frame_center_z)),
        material=body_material,
        name="bottom_rail",
    )
    part.visual(
        Box((0.008, 0.114, frame_height)),
        origin=Origin(xyz=(-0.141, 0.000, frame_center_z)),
        material=body_material,
        name="left_endcap",
    )
    part.visual(
        Box((0.014, 0.114, frame_height)),
        origin=Origin(xyz=(0.144, 0.000, frame_center_z)),
        material=body_material,
        name="right_endcap",
    )
    part.visual(
        Box((0.008, 0.066, frame_height)),
        origin=Origin(xyz=(-0.046, 0.000, frame_center_z)),
        material=body_material,
        name="bridge_left",
    )
    part.visual(
        Box((0.008, 0.066, frame_height)),
        origin=Origin(xyz=(0.046, 0.000, frame_center_z)),
        material=body_material,
        name="bridge_right",
    )

    for label, x_pos in zip(("left", "center", "right"), FAN_CENTERS):
        part.visual(
            bezel_mesh,
            origin=Origin(xyz=(x_pos, 0.000, 0.030)),
            material=body_material,
            name=f"bezel_{label}",
        )
        part.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x_pos, 0.000, 0.025)),
            material=accent_material,
            name=f"bearing_{label}",
        )
        part.visual(
            Box((0.006, 0.034, 0.004)),
            origin=Origin(xyz=(x_pos, 0.027, 0.025)),
            material=accent_material,
            name=f"bearing_top_strut_{label}",
        )
        part.visual(
            Box((0.006, 0.034, 0.004)),
            origin=Origin(xyz=(x_pos, -0.027, 0.025)),
            material=accent_material,
            name=f"bearing_bottom_strut_{label}",
        )

    part.visual(
        Box((0.074, 0.008, 0.006)),
        origin=Origin(xyz=(-0.088, 0.028, 0.038)),
        material=accent_material,
        name="accent_upper_left",
    )
    part.visual(
        Box((0.074, 0.008, 0.006)),
        origin=Origin(xyz=(0.088, 0.028, 0.038)),
        material=accent_material,
        name="accent_upper_right",
    )
    part.visual(
        Box((0.074, 0.008, 0.006)),
        origin=Origin(xyz=(-0.088, -0.028, 0.038)),
        material=accent_material,
        name="accent_lower_left",
    )
    part.visual(
        Box((0.074, 0.008, 0.006)),
        origin=Origin(xyz=(0.088, -0.028, 0.038)),
        material=accent_material,
        name="accent_lower_right",
    )
    part.visual(
        Box((0.004, 0.010, 0.004)),
        origin=Origin(xyz=(FAN_CENTERS[1] + 0.0445, 0.000, 0.034)),
        material=accent_material,
        name="center_ref_x",
    )
    part.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(FAN_CENTERS[1], 0.0445, 0.034)),
        material=accent_material,
        name="center_ref_y",
    )


def _add_backplate_details(part, plate_material, accent_material) -> None:
    part.visual(
        Box((0.294, 0.110, BACKPLATE_THICKNESS)),
        origin=Origin(xyz=(0.003, 0.000, -BACKPLATE_THICKNESS * 0.5)),
        material=plate_material,
        name="plate",
    )
    for index, y in enumerate((-0.030, 0.000, 0.030)):
        part.visual(
            Box((0.240, 0.006, 0.003)),
            origin=Origin(xyz=(0.018, y, -0.0022)),
            material=accent_material,
            name=f"rib_{index}",
        )
    part.visual(
        Box((0.050, 0.024, 0.0032)),
        origin=Origin(xyz=(-0.106, 0.000, -0.0023)),
        material=accent_material,
        name="vrm_plate",
    )
    part.visual(
        Box((0.070, 0.014, 0.0025)),
        origin=Origin(xyz=(0.120, 0.038, -0.00205)),
        material=accent_material,
        name="logo_bar",
    )


def _add_bracket_details(part, bracket_material, dark_material) -> None:
    plate_x = -0.149
    part.visual(
        Box((0.002, 0.112, 0.004)),
        origin=Origin(xyz=(plate_x, 0.000, 0.018)),
        material=bracket_material,
        name="front_edge",
    )
    part.visual(
        Box((0.002, 0.112, 0.004)),
        origin=Origin(xyz=(plate_x, 0.000, 0.002)),
        material=bracket_material,
        name="rear_edge",
    )
    part.visual(
        Box((0.002, 0.010, 0.020)),
        origin=Origin(xyz=(plate_x, 0.051, 0.010)),
        material=bracket_material,
        name="top_cross",
    )
    part.visual(
        Box((0.002, 0.010, 0.020)),
        origin=Origin(xyz=(plate_x, 0.000, 0.010)),
        material=bracket_material,
        name="middle_cross",
    )
    part.visual(
        Box((0.002, 0.010, 0.020)),
        origin=Origin(xyz=(plate_x, -0.051, 0.010)),
        material=bracket_material,
        name="bottom_cross",
    )
    part.visual(
        Box((0.010, 0.020, 0.002)),
        origin=Origin(xyz=(-0.144, -0.040, 0.003)),
        material=bracket_material,
        name="mount_tongue",
    )
    part.visual(
        Box((0.002, 0.018, 0.012)),
        origin=Origin(xyz=(plate_x, 0.062, 0.020)),
        material=bracket_material,
        name="screw_ear",
    )
    part.visual(
        Box((0.002, 0.038, 0.014)),
        origin=Origin(xyz=(plate_x, 0.026, 0.010)),
        material=dark_material,
        name="upper_port_liner",
    )
    part.visual(
        Box((0.002, 0.038, 0.014)),
        origin=Origin(xyz=(plate_x, -0.026, 0.010)),
        material=dark_material,
        name="lower_port_liner",
    )


def _build_fan_part(model, name: str, fan_material, hub_material, rim_mesh, *, add_tab: bool):
    fan = model.part(name)
    fan.visual(
        rim_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.034)),
        material=fan_material,
        name="rim",
    )
    fan.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.033)),
        material=hub_material,
        name="hub",
    )
    fan.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
        material=hub_material,
        name="axle",
    )
    fan.visual(
        Cylinder(radius=0.0095, length=0.0025),
        origin=Origin(xyz=(0.000, 0.000, 0.03725)),
        material=fan_material,
        name="cap",
    )

    blade_count = 9
    for index in range(blade_count):
        angle = index * math.tau / blade_count
        fan.visual(
            Box((0.028, 0.008, 0.0026)),
            origin=Origin(
                xyz=(0.024 * math.cos(angle), 0.024 * math.sin(angle), 0.033),
                rpy=(0.22, 0.00, angle + 0.55),
            ),
            material=fan_material,
            name=f"blade_{index}",
        )

    if add_tab:
        fan.visual(
            Box((0.004, 0.006, 0.002)),
            origin=Origin(xyz=(0.0365, 0.000, 0.034)),
            material=hub_material,
            name="timing_tab",
        )

    fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.012),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.000, 0.033)),
    )
    return fan


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="graphics_card", assets=ASSETS)

    shroud_black = model.material("shroud_black", rgba=(0.12, 0.13, 0.14, 1.0))
    shroud_trim = model.material("shroud_trim", rgba=(0.20, 0.22, 0.24, 1.0))
    pcb_black = model.material("pcb_black", rgba=(0.07, 0.09, 0.07, 1.0))
    fin_aluminum = model.material("fin_aluminum", rgba=(0.55, 0.58, 0.62, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.66, 0.68, 0.71, 1.0))
    backplate_metal = model.material("backplate_metal", rgba=(0.30, 0.33, 0.36, 1.0))
    fan_black = model.material("fan_black", rgba=(0.10, 0.10, 0.11, 1.0))
    gold = model.material("gold", rgba=(0.82, 0.69, 0.25, 1.0))

    bezel_mesh = _save_mesh(
        _ring_band_mesh(
            outer_radius=SHROUD_BEZEL_OUTER,
            inner_radius=SHROUD_BEZEL_INNER,
            height=SHROUD_TOP_Z - SHROUD_BOTTOM_Z,
        ),
        "gpu_shroud_bezel.obj",
    )
    rim_mesh = _save_mesh(
        _ring_band_mesh(
            outer_radius=FAN_RIM_OUTER,
            inner_radius=FAN_RIM_INNER,
            height=0.006,
        ),
        "gpu_fan_rim.obj",
    )

    core = model.part("core")
    _add_core_details(core, pcb_black, shroud_trim, gold)
    _add_fin_stack(core, fin_aluminum)
    core.inertial = Inertial.from_geometry(
        Box((0.300, 0.118, 0.024)),
        mass=1.45,
        origin=Origin(xyz=(0.005, 0.000, 0.012)),
    )

    shroud = model.part("shroud")
    _add_shroud_structure(shroud, shroud_black, shroud_trim, bezel_mesh)
    shroud.inertial = Inertial.from_geometry(
        Box((0.292, 0.116, SHROUD_TOP_Z - SHROUD_BOTTOM_Z)),
        mass=0.42,
        origin=Origin(xyz=(0.002, 0.000, 0.030)),
    )

    backplate = model.part("backplate")
    _add_backplate_details(backplate, backplate_metal, shroud_trim)
    backplate.inertial = Inertial.from_geometry(
        Box((0.296, 0.112, 0.004)),
        mass=0.28,
        origin=Origin(xyz=(0.003, 0.000, -0.002)),
    )

    bracket = model.part("bracket")
    _add_bracket_details(bracket, bracket_metal, shroud_trim)
    bracket.inertial = Inertial.from_geometry(
        Box((0.012, 0.118, 0.026)),
        mass=0.08,
        origin=Origin(xyz=(-0.145, 0.000, 0.013)),
    )

    fan_left = _build_fan_part(
        model,
        "fan_left",
        fan_black,
        shroud_trim,
        rim_mesh,
        add_tab=False,
    )
    fan_center = _build_fan_part(
        model,
        "fan_center",
        fan_black,
        shroud_trim,
        rim_mesh,
        add_tab=True,
    )
    fan_right = _build_fan_part(
        model,
        "fan_right",
        fan_black,
        shroud_trim,
        rim_mesh,
        add_tab=False,
    )

    model.articulation(
        "core_to_shroud",
        ArticulationType.FIXED,
        parent=core,
        child=shroud,
        origin=Origin(),
    )
    model.articulation(
        "core_to_backplate",
        ArticulationType.FIXED,
        parent=core,
        child=backplate,
        origin=Origin(),
    )
    model.articulation(
        "core_to_bracket",
        ArticulationType.FIXED,
        parent=core,
        child=bracket,
        origin=Origin(),
    )

    for label, part, x_pos in (
        ("left", fan_left, FAN_CENTERS[0]),
        ("center", fan_center, FAN_CENTERS[1]),
        ("right", fan_right, FAN_CENTERS[2]),
    ):
        model.articulation(
            f"shroud_to_fan_{label}",
            ArticulationType.CONTINUOUS,
            parent=shroud,
            child=part,
            origin=Origin(xyz=(x_pos, 0.000, FAN_Z_JOINT)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.3, velocity=50.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    core = object_model.get_part("core")
    shroud = object_model.get_part("shroud")
    backplate = object_model.get_part("backplate")
    bracket = object_model.get_part("bracket")
    fan_left = object_model.get_part("fan_left")
    fan_center = object_model.get_part("fan_center")
    fan_right = object_model.get_part("fan_right")

    shroud_to_fan_left = object_model.get_articulation("shroud_to_fan_left")
    shroud_to_fan_center = object_model.get_articulation("shroud_to_fan_center")
    shroud_to_fan_right = object_model.get_articulation("shroud_to_fan_right")

    pcb = core.get_visual("pcb")
    fin_base = core.get_visual("fin_base")
    top_spreader = core.get_visual("top_spreader")
    backplate_plate = backplate.get_visual("plate")
    bracket_tongue = bracket.get_visual("mount_tongue")
    bearing_left = shroud.get_visual("bearing_left")
    bearing_center = shroud.get_visual("bearing_center")
    bearing_right = shroud.get_visual("bearing_right")
    bezel_left = shroud.get_visual("bezel_left")
    bezel_center = shroud.get_visual("bezel_center")
    bezel_right = shroud.get_visual("bezel_right")
    center_ref_x = shroud.get_visual("center_ref_x")
    center_ref_y = shroud.get_visual("center_ref_y")
    fan_left_axle = fan_left.get_visual("axle")
    fan_center_axle = fan_center.get_visual("axle")
    fan_right_axle = fan_right.get_visual("axle")
    fan_left_rim = fan_left.get_visual("rim")
    fan_center_rim = fan_center.get_visual("rim")
    fan_right_rim = fan_right.get_visual("rim")
    fan_center_tab = fan_center.get_visual("timing_tab")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual connectivity gate for floating/disconnected subassemblies inside one part.
    ctx.fail_if_part_contains_disconnected_geometry_islands()
    # Default broad part-level rest-pose backstop for top-level interpenetration.
    # If a seated or nested fit is intentional, justify it with `ctx.allow_overlap(...)`.
    ctx.fail_if_parts_overlap_in_current_pose()

    # Encode the actual visual/mechanical claims with prompt-specific exact checks.
    # If you add a warning-tier heuristic and it fires, investigate it with
    # `probe_model` before editing geometry or relaxing thresholds.
    # Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is
    # genuinely uncertain or mechanically important.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # For ctx.expect_* helpers, keep the first body/link arguments as Part objects.
    # Named Visuals belong only in elem_a/elem_b/positive_elem/negative_elem/inner_elem/outer_elem.
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # If the object has a mounted subassembly, prefer exact `expect_contact(...)`,
    # `expect_gap(...)`, `expect_overlap(...)`, and `expect_within(...)` checks on
    # named local features over the broad rest-pose overlap backstop.
    ctx.expect_overlap(shroud, core, axes="xy", min_overlap=0.09, elem_b=top_spreader)
    ctx.expect_origin_distance(shroud, core, axes="xy", max_dist=0.008)
    ctx.expect_gap(
        shroud,
        core,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=shroud.get_visual("top_rail"),
        negative_elem=top_spreader,
    )

    ctx.expect_within(
        core,
        backplate,
        axes="xy",
        inner_elem=pcb,
        outer_elem=backplate_plate,
    )
    ctx.expect_origin_distance(backplate, core, axes="xy", max_dist=0.004)
    ctx.expect_overlap(shroud, backplate, axes="xy", min_overlap=0.10)
    ctx.expect_gap(
        core,
        backplate,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=pcb,
        negative_elem=backplate_plate,
    )

    ctx.expect_gap(
        core,
        bracket,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=pcb,
        negative_elem=bracket_tongue,
    )
    ctx.expect_gap(
        bracket,
        core,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=bracket_tongue,
        negative_elem=pcb,
    )
    ctx.expect_overlap(bracket, core, axes="y", min_overlap=0.018, elem_a=bracket_tongue, elem_b=pcb)

    for fan_part, fan_axle, fan_rim, bearing, bezel in (
        (fan_left, fan_left_axle, fan_left_rim, bearing_left, bezel_left),
        (fan_center, fan_center_axle, fan_center_rim, bearing_center, bezel_center),
        (fan_right, fan_right_axle, fan_right_rim, bearing_right, bezel_right),
    ):
        ctx.expect_gap(
            fan_part,
            shroud,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-6,
            positive_elem=fan_axle,
            negative_elem=bearing,
        )
        ctx.expect_within(
            fan_part,
            shroud,
            axes="xy",
            inner_elem=fan_rim,
            outer_elem=bezel,
        )
        ctx.expect_overlap(fan_part, shroud, axes="xy", min_overlap=0.07, elem_a=fan_rim, elem_b=bezel)

    ctx.expect_origin_distance(fan_left, fan_center, axes="y", max_dist=0.001)
    ctx.expect_origin_distance(fan_right, fan_center, axes="y", max_dist=0.001)
    ctx.expect_origin_distance(fan_left, fan_right, axes="yz", max_dist=0.001)

    ctx.expect_gap(
        shroud,
        fan_center,
        axis="x",
        max_gap=0.0045,
        max_penetration=0.0,
        positive_elem=center_ref_x,
        negative_elem=fan_center_tab,
    )
    with ctx.pose({shroud_to_fan_center: math.pi / 2.0}):
        ctx.expect_gap(
            shroud,
            fan_center,
            axis="y",
            max_gap=0.0045,
            max_penetration=0.0,
            positive_elem=center_ref_y,
            negative_elem=fan_center_tab,
        )
        ctx.expect_within(
            fan_center,
            shroud,
            axes="xy",
            inner_elem=fan_center_rim,
            outer_elem=bezel_center,
        )

    with ctx.pose(
        {
            shroud_to_fan_left: math.pi / 3.0,
            shroud_to_fan_center: -math.pi / 4.0,
            shroud_to_fan_right: math.pi / 2.5,
        }
    ):
        for fan_part, fan_axle, fan_rim, bearing, bezel in (
            (fan_left, fan_left_axle, fan_left_rim, bearing_left, bezel_left),
            (fan_center, fan_center_axle, fan_center_rim, bearing_center, bezel_center),
            (fan_right, fan_right_axle, fan_right_rim, bearing_right, bezel_right),
        ):
            ctx.expect_gap(
                fan_part,
                shroud,
                axis="z",
                max_gap=0.001,
                max_penetration=1e-6,
                positive_elem=fan_axle,
                negative_elem=bearing,
            )
            ctx.expect_within(
                fan_part,
                shroud,
                axes="xy",
                inner_elem=fan_rim,
                outer_elem=bezel,
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
