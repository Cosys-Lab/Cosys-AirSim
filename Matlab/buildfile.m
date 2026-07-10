function plan = buildfile
    import matlab.buildtool.tasks.CleanTask
    import matlab.buildtool.tasks.CodeIssuesTask

    % Create a plan from the task functions
    plan = buildplan(localfunctions);
    
    % Define the "clean" Task
    plan("clean") = matlab.buildtool.tasks.CleanTask;    
    
    % Define the "check" task
    sourceFolder = files(plan, "toolbox");
    plan("check") = matlab.buildtool.tasks.CodeIssuesTask(sourceFolder,...
        IncludeSubfolders = true);
    
    plan.DefaultTasks = ["clean" "check" "generatedocs" "release"];

    % Make the "release" task dependent on the others
    plan("release").Dependencies = ["check" "generatedocs"];
    plan("release").Outputs = "release\sonotraceue.mltbx";
end

function releaseTask(~)
    % Create an MLTBX package
    releaseFolderName = "release";
    % Create a release and put it in the release directory
    opts = matlab.addons.toolbox.ToolboxOptions("SonoTraceUE-Toolbox.prj");
    
    % By default, the packaging GUI restricts the name of the getting started guide, so we fix that here.
    opts.ToolboxGettingStartedGuide = fullfile("toolbox", "doc", "gettingStarted.mlx");
    
    % GitHub releases don't allow spaces, so replace spaces with underscores
    opts.OutputFile = fullfile(releaseFolderName, "sonotraceue.mltbx");
    
    % Create the release directory, if needed
    if ~exist(releaseFolderName,"dir")
        mkdir(releaseFolderName)
    end
    matlab.addons.toolbox.packageToolbox(opts);
end

function generatedocsTask(~)
    % Generate markdown readme
    
    mdfile = export("toolbox/doc/GettingStarted.mlx", "README.md", Format="markdown");
    
    % Clean up the README.md file
    cleanupReadme("README.md");
    
    % Generate HTML pages
    htmldir = "toolbox\doc\html";
    if ~exist(htmldir, 'dir')
        mkdir(htmldir)
    end
    mdfile = export("toolbox/doc/GettingStarted.mlx", "toolbox/doc/html/GettingStarted.html", Format="html");
end

function cleanupReadme(filename)
    % Read the file
    fileContent = fileread(filename);
    lines = splitlines(fileContent);
    
    % Find and remove TOC section (between <!-- Begin Toc --> and <!-- End Toc -->)
    inToc = false;
    linesToKeep = true(size(lines));
    
    for i = 1:length(lines)
        if contains(lines{i}, '<!-- Begin Toc -->')
            inToc = true;
            linesToKeep(i) = false;
        elseif contains(lines{i}, '<!-- End Toc -->')
            inToc = false;
            linesToKeep(i) = false;
        elseif inToc
            linesToKeep(i) = false;
        end
    end
    
    lines = lines(linesToKeep);
    
    % Remove lines containing problematic links
    % 1. Lines with MATLAB-specific anchor links like [text](#H_xxxx)
    % 2. For lines with links to .mlx/.m files, remove only sentences with those links
    linesToKeep = true(size(lines));
    
    for i = 1:length(lines)
        line = lines{i};
        % Check for MATLAB anchor links pattern: [text](#H_xxxx) or [text](#TMP_xxxx)
        if ~isempty(regexp(line, '\[.*?\]\(#[HT]_[a-zA-Z0-9]+\)', 'once'))
            linesToKeep(i) = false;
            continue;
        end
        
        % Check for links to .mlx or .m files
        if contains(line, '.mlx)') || contains(line, '.m)')
            % Split line into sentences (split by period followed by space or end of string)
            sentences = regexp(line, '[^.]*\.(?:\s|$)', 'match');
            
            % If no sentences found (no periods), check the whole line
            if isempty(sentences)
                sentences = {line};
            end
            
            cleanSentences = {};
            for j = 1:length(sentences)
                sentence = sentences{j};
                % Keep sentence only if it doesn't contain .mlx or .m links
                if ~contains(sentence, '.mlx)') && ~contains(sentence, '.m)')
                    cleanSentences{end+1} = sentence;
                end
            end
            
            % Reconstruct the line if there are any clean sentences
            if ~isempty(cleanSentences)
                lines{i} = strjoin(cleanSentences, '');
                % Trim any extra whitespace
                lines{i} = strtrim(lines{i});
            else
                % If all sentences had problematic links, mark line for removal
                linesToKeep(i) = false;
            end
        end
    end
    
    lines = lines(linesToKeep);
    
    % Downgrade heading levels (except the first main title)
    % MATLAB exports title as #, but we want to keep first heading as # and downgrade all others
    firstHeadingFound = false;
    for i = 1:length(lines)
        line = lines{i};
        % Check if line is a markdown heading (starts with #)
        if ~isempty(line) && line(1) == '#'
            if ~firstHeadingFound
                % Keep the first heading as-is (the main title)
                firstHeadingFound = true;
            else
                % Downgrade all subsequent headings by one level (add one more #)
                lines{i} = ['#' line];
            end
        end
    end
    
    % Write the cleaned content back to file
    fileContent = strjoin(lines, newline);
    fid = fopen(filename, 'w', 'n', 'UTF-8');
    fwrite(fid, fileContent, 'char');
    fclose(fid);
end